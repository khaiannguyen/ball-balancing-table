#ifndef TRAJECTORY_HPP

#define TRAJECTORY_HPP

#include <array>
#include <cstddef>
#include <cmath>

/*
 * Generates the setpoint trajectory used by BALANCE mode.
 *
 * The trajectory is organized as a sequence of fixed waypoints followed by
 * continuous geometric paths. The controller returns a Waypoint to the
 * control loop, which is responsible for tracking the generated setpoint.
 *
 * Main trajectory sequence:
 *
 *   O -> C -> A -> E -> C
 *      -> F -> B -> D -> F
 *      -> A -> B -> C -> D -> E -> F
 *      -> CIRCLE
 *      -> FIGURE8
 *      -> O
 *
 * Fixed waypoints are held for a configurable dwell time. Short transit
 * points are inserted between consecutive waypoints to reduce abrupt
 * setpoint changes and keep the physical trajectory closer to the intended
 * straight-line path.
 *
 * The CIRCLE phase uses a continuous open-loop angular trajectory. The
 * FIGURE8 phase generates two horizontal and two vertical figure-eight loops
 * around the center.
 *
 * All positions are expressed in millimeters in the robot/table coordinate
 * frame.
 */
class BalanceTrajectoryController
{
public:
    /*
     * Height is currently unused by the trajectory engine.
     *
     * The field is retained for compatibility with the control-loop data
     * structure, which expects a three-axis waypoint representation.
     */
    struct Waypoint
    {
        float x_mm = 0.f;
        float y_mm = 0.f;
        float height_mm = 0.f;
    };

    BalanceTrajectoryController() = default;

    /*
     * Resets the trajectory to the initial center point.
     *
     * This is used when entering BALANCE mode or when the ball becomes
     * available again after being lost. Restarting from O provides a known
     * and deterministic trajectory state.
     */
    void reset()
    {
        seq_index_ = 0;
        sub_phase_ = SubPhase::AT_POINT;
        phase_elapsed_s_ = 0.f;
        transit_step_ = 0;
        circle_angle_traveled_ = 0.f;
        figure8_angle_traveled_ = 0.f;
        figure8_loop_angle_ = 0.f;
        figure8_loop_index_ = 0;
        figure8_horizontal_ = true;
    }

    /*
     * Advances the trajectory by dt seconds and returns the current setpoint.
     *
     * Fixed-point phases return the active waypoint. Transit phases return
     * interpolated points between two consecutive waypoints. Continuous
     * phases return analytically generated positions.
     */
    Waypoint update(float dt)
    {
        phase_elapsed_s_ += dt;

        const std::size_t next_index =
            (seq_index_ + 1) % kNumPoints;

        switch (sub_phase_)
        {
        case SubPhase::AT_POINT:
        {
            const float dwell =
                (seq_index_ == 0 && post_figure8_hold_)
                ? kPostFigure8HoldSeconds
                : dwellSecondsFor(seq_index_);

            if (phase_elapsed_s_ >= dwell)
            {
                if (seq_index_ == 0)
                {
                    post_figure8_hold_ = false;
                }

                /*
                 * The final waypoint transitions directly into the circular
                 * phase instead of wrapping immediately back to O.
                 */
                if (seq_index_ == kNumPoints - 1)
                {
                    sub_phase_ = SubPhase::CIRCLE;
                    circle_angle_traveled_ = 0.f;
                }
                else
                {
                    sub_phase_ = SubPhase::TRANSIT;
                }

                phase_elapsed_s_ = 0.f;
            }

            return kSequence[seq_index_];
        }

        case SubPhase::TRANSIT:
        {
            /*
             * Three intermediate points are inserted at 25%, 50%, and 75%
             * of the distance between consecutive primary waypoints.
             *
             * Each point is held briefly to allow the control loop to follow
             * the intended straight-line path without a large setpoint jump.
             */
            if (phase_elapsed_s_ >= kTransitSeconds)
            {
                phase_elapsed_s_ = 0.f;

                ++transit_step_;

                if (transit_step_ >= kNumTransitPoints)
                {
                    transit_step_ = 0;
                    seq_index_ = next_index;
                    sub_phase_ = SubPhase::AT_POINT;

                    return kSequence[seq_index_];
                }
            }

            const Waypoint& a =
                kSequence[seq_index_];

            const Waypoint& b =
                kSequence[next_index];

            const float frac =
                static_cast<float>(transit_step_ + 1) /
                static_cast<float>(kNumTransitPoints + 1);

            return Waypoint{
                a.x_mm + (b.x_mm - a.x_mm) * frac,
                a.y_mm + (b.y_mm - a.y_mm) * frac,
                0.f
            };
        }

        case SubPhase::CIRCLE:
        {
            /*
             * The phase timer is already updated at the beginning of
             * update(), so phase_elapsed_s_ must not be incremented again.
             */
            const float time_in_circle =
                phase_elapsed_s_;

            circle_angle_traveled_ +=
                kCircleAngularSpeed * dt;

            if (circle_angle_traveled_ >= kCircleTotalAngle)
            {
                /*
                 * Transition directly to FIGURE8. The trajectory returns to
                 * the center as the first FIGURE8 point without an additional
                 * dwell phase.
                 */
                circle_angle_traveled_ =
                    kCircleTotalAngle;

                sub_phase_ =
                    SubPhase::FIGURE8;

                phase_elapsed_s_ = 0.f;
                figure8_angle_traveled_ = 0.f;
                figure8_loop_angle_ = 0.f;
                figure8_loop_index_ = 0;
                figure8_horizontal_ = true;

                return Waypoint{
                    0.f,
                    0.f,
                    0.f
                };
            }

            const float angle =
                kCircleStartAngle +
                circle_angle_traveled_;

            const float time_remaining =
                kCircleSeconds -
                time_in_circle;

            const float ramp_in =
                clamp01(
                    time_in_circle /
                    kCircleRampSeconds
                );

            const float ramp_out =
                clamp01(
                    time_remaining /
                    kCircleRampSeconds
                );

            const float effective_radius =
                kCircleRadius *
                std::min(
                    ramp_in,
                    ramp_out
                );

            return Waypoint{
                effective_radius * std::cos(angle),
                effective_radius * std::sin(angle),
                0.f
            };
        }

        case SubPhase::FIGURE8:
        {
            /*
             * Four continuous figure-eight loops are generated:
             *
             *   loop 0-1: horizontal
             *   loop 2-3: vertical
             *
             * Each loop covers exactly 2*pi radians and returns to the
             * center, allowing the next loop to start from a known position.
             */
            const float dt_now = dt;

            figure8_loop_angle_ +=
                kFigure8AngularSpeed * dt_now;

            figure8_angle_traveled_ +=
                kFigure8AngularSpeed * dt_now;

            while (
                figure8_loop_angle_ >= 2.f * kPi &&
                figure8_loop_index_ < 4)
            {
                figure8_loop_angle_ -=
                    2.f * kPi;

                ++figure8_loop_index_;
            }

            if (figure8_loop_index_ >= 4)
            {
                /*
                 * All four loops are complete. Return to O and hold briefly
                 * before restarting the main waypoint sequence.
                 */
                seq_index_ = 0;
                sub_phase_ = SubPhase::AT_POINT;
                phase_elapsed_s_ = 0.f;
                transit_step_ = 0;
                post_figure8_hold_ = true;

                figure8_angle_traveled_ =
                    kFigure8TotalAngle;

                figure8_loop_angle_ = 0.f;
                figure8_loop_index_ = 4;

                return kSequence[0];
            }

            /*
             * Each figure-eight starts at t = pi/2, which places the
             * trajectory at the center point O.
             */
            const float t =
                kFigure8StartAngle +
                figure8_loop_angle_;

            /*
             * Ramp the amplitude in only at the beginning of the phase.
             *
             * Each individual figure-eight naturally returns to O, so a
             * global ramp-out at the end of the full phase is unnecessary.
             */
            const float ramp_in =
                clamp01(
                    phase_elapsed_s_ /
                    kFigure8RampSeconds
                );

            const float effective_amp =
                kFigure8Amplitude *
                ramp_in;

            const float s8 =
                std::sin(t);

            const float c8 =
                std::cos(t);

            if (figure8_loop_index_ < 2)
            {
                /*
                 * First two loops: horizontal figure-eight.
                 */
                return Waypoint{
                    effective_amp * c8,
                    effective_amp * s8 * c8,
                    0.f
                };
            }
            else
            {
                /*
                 * Last two loops: vertical figure-eight.
                 */
                return Waypoint{
                    effective_amp * s8 * c8,
                    effective_amp * c8,
                    0.f
                };
            }
        }
        }

        /*
         * This path is unreachable when sub_phase_ contains a valid value.
         * Keep O as a deterministic fallback.
         */
        return kSequence[0];
    }

private:
    enum class SubPhase
    {
        AT_POINT,
        TRANSIT,
        CIRCLE,
        FIGURE8
    };

    /*
     * Hexagon geometry in millimeters.
     *
     * Coordinate convention:
     *
     *   A = upper-left
     *   B = upper-right
     *   C = right
     *   D = lower-right
     *   E = lower-left
     *   F = left
     *   O = center
     *
     * If the physical table geometry differs from these assumptions, update
     * the six waypoint coordinates below without changing the trajectory
     * state machine.
     */
    static constexpr float kHexRadius = 125.0f;

    static constexpr float kA_x = 120.25f;
    static constexpr float kA_y = -62.5f;

    static constexpr float kB_x = 120.25f;
    static constexpr float kB_y = 62.5f;

    static constexpr float kC_x = 0.0f;
    static constexpr float kC_y = 125.0f;

    static constexpr float kD_x = -108.25f;
    static constexpr float kD_y = 62.5f;

    static constexpr float kE_x = -108.25f;
    static constexpr float kE_y = -62.5f;

    static constexpr float kF_x = 0.0f;
    static constexpr float kF_y = -125.0f;

    static constexpr float kO_x = 0.0f;
    static constexpr float kO_y = 0.0f;

    /*
     * Optional test mode that bypasses the normal waypoint sequence and
     * starts directly from the center before entering the circular phase.
     *
     * Set to 1 only when the circular trajectory is being tested in isolation.
     */
#define TRAJ_TEST_CIRCLE_ONLY 0

     /*
      * Main waypoint sequence:
      *
      * O -> C -> A -> E -> C
      * -> F -> B -> D -> F
      * -> A -> B -> C -> D -> E -> F
      *
      * The sequence wraps back to O after the final waypoint.
      */
#if TRAJ_TEST_BOUNCE_ONLY

      /*
       * Bounce-only test configuration.
       *
       * This mode retains only O as the primary waypoint and relies on the
       * corresponding update() transition logic for the test trajectory.
       */
    static constexpr std::size_t kNumPoints = 1;

    static constexpr std::array<
        Waypoint,
        kNumPoints
    > kSequence = { {
        {kO_x, kO_y, 0.f}
    } };

#elif TRAJ_TEST_CIRCLE_ONLY

      /*
       * Circle-only test configuration.
       *
       * O is the only primary waypoint. Because it is also the final waypoint,
       * the normal AT_POINT transition enters the CIRCLE phase directly.
       *
       * This avoids a large setpoint transition from the outer hexagon into the
       * smaller circular trajectory.
       */
    static constexpr std::size_t kNumPoints = 1;

    static constexpr std::array<
        Waypoint,
        kNumPoints
    > kSequence = { {
        {kO_x, kO_y, 0.f}
    } };

#else

    static constexpr std::size_t kNumPoints = 15;

    static constexpr std::array<
        Waypoint,
        kNumPoints
    > kSequence = { {
        {kO_x, kO_y, 0.f},   // 0: O
        {kC_x, kC_y, 0.f},   // 1: C
        {kA_x, kA_y, 0.f},   // 2: A
        {kE_x, kE_y, 0.f},   // 3: E
        {kC_x, kC_y, 0.f},   // 4: C
        {kF_x, kF_y, 0.f},   // 5: F
        {kB_x, kB_y, 0.f},   // 6: B
        {kD_x, kD_y, 0.f},   // 7: D
        {kF_x, kF_y, 0.f},   // 8: F
        {kA_x, kA_y, 0.f},   // 9: A
        {kB_x, kB_y, 0.f},   // 10: B
        {kC_x, kC_y, 0.f},   // 11: C
        {kD_x, kD_y, 0.f},   // 12: D
        {kE_x, kE_y, 0.f},   // 13: E
        {kF_x, kF_y, 0.f}    // 14: F
    } };

#endif

    /*
     * Center dwell time.
     */
    static constexpr float kHoldCenterSeconds = 2.0f;

    /*
     * Dwell time at each non-center primary waypoint.
     *
     * The control loop is given enough time to reduce transient error before
     * the next setpoint transition.
     */
    static constexpr float kDwellSeconds = 2.0f;

    /*
     * Short center hold after FIGURE8 before restarting O -> C.
     */
    static constexpr float kPostFigure8HoldSeconds = 0.25f;

    /*
     * Hold time at each interpolated transit point.
     *
     * Transit points are intended to shape the physical path rather than
     * provide additional stabilization time.
     */
    static constexpr float kTransitSeconds = 0.2f;

    /*
     * Number of intermediate points inserted between consecutive primary
     * waypoints.
     *
     * With three points, the generated fractions are 0.25, 0.5, and 0.75.
     */
    static constexpr int kNumTransitPoints = 3;

    static float dwellSecondsFor(
        std::size_t index)
    {
        return (index == 0)
            ? kHoldCenterSeconds
            : kDwellSeconds;
    }

    /*
     * Continuous circular trajectory around O.
     *
     * The path is generated open-loop using angular position rather than
     * discrete waypoints. The radius is ramped at the beginning and end to
     * avoid abrupt radial acceleration.
     */
    static constexpr float kPi =
        3.14159265358979f;

    /*
     * Circle radius in millimeters.
     *
     * Keep this below the physical table boundary to provide margin for
     * tracking error and overshoot.
     */
    static constexpr float kCircleRadius =
        120.0f;

    /*
     * Number of complete revolutions in the circular phase.
     */
    static constexpr int kCircleLaps = 2;

    /*
     * Total duration of the circular phase.
     *
     * The resulting angular speed is:
     *
     *   angular_speed = total_angle / duration
     *
     * A slower angular speed gives the PID controller more time to track
     * the moving setpoint.
     */
    static constexpr float kCircleSeconds =
        10.0f;

    static constexpr float kCircleTotalAngle =
        2.0f *
        kPi *
        static_cast<float>(
            kCircleLaps
            );

    static constexpr float kCircleAngularSpeed =
        kCircleTotalAngle /
        kCircleSeconds;

    /*
     * Radius ramp duration at the beginning and end of the circular phase.
     *
     * The combined ramp duration must remain shorter than the total circle
     * duration.
     */
    static constexpr float kCircleRampSeconds =
        0.75f;

    static float clamp01(
        float v)
    {
        return v < 0.f
            ? 0.f
            : (v > 1.f ? 1.f : v);
    }

    /*
     * Circle start angle.
     *
     * The trajectory starts at the former F direction:
     *
     *   F = (0, -125 mm)
     *
     * which corresponds to -pi/2 radians in the table coordinate frame.
     */
    static constexpr float kCircleStartAngle =
        -kPi * 0.5f;

    /*
     * Continuous figure-eight trajectory around O.
     *
     * This phase replaces the previous bounce trajectory. Four complete
     * figure-eight loops are generated: two horizontal followed by two
     * vertical.
     */
    static constexpr float kFigure8StartAngle =
        kPi * 0.5f;

    /*
     * Figure-eight amplitude in millimeters.
     *
     * The amplitude is intentionally smaller than the hexagon radius to keep
     * the generated path inside the physical working area.
     */
    static constexpr float kFigure8Amplitude =
        90.0f;

    /*
     * Total number of figure-eight loops:
     *
     *   2 horizontal
     *   2 vertical
     */
    static constexpr int kFigure8Laps = 4;

    /*
     * Total duration of the figure-eight phase.
     *
     * Angular speed is derived from the total angle and this duration.
     */
    static constexpr float kFigure8Seconds =
        24.0f;

    static constexpr float kFigure8TotalAngle =
        2.0f *
        kPi *
        static_cast<float>(
            kFigure8Laps
            );

    static constexpr float kFigure8AngularSpeed =
        kFigure8TotalAngle /
        kFigure8Seconds;

    /*
     * Initial amplitude ramp duration for the figure-eight phase.
     *
     * The trajectory starts at O and gradually increases its amplitude to
     * avoid an abrupt transition from the preceding phase.
     */
    static constexpr float kFigure8RampSeconds =
        0.30f;

    std::size_t seq_index_ = 0;

    SubPhase sub_phase_ =
        SubPhase::AT_POINT;

    float phase_elapsed_s_ = 0.f;

    /*
     * Current transit point index:
     *   0 .. kNumTransitPoints - 1
     */
    int transit_step_ = 0;

    /*
     * Adds a short center hold after FIGURE8 before restarting O -> C.
     */
    bool post_figure8_hold_ = false;

    /*
     * Indicates the orientation of the current figure-eight group.
     */
    bool figure8_horizontal_ = true;

    /*
     * Figure-eight loop index:
     *   0..1 = horizontal
     *   2..3 = vertical
     */
    int figure8_loop_index_ = 0;

    /*
     * Local angular position within the current figure-eight loop.
     */
    float figure8_loop_angle_ = 0.f;

    /*
     * Accumulated angular travel during the circular phase.
     */
    float circle_angle_traveled_ = 0.f;

    /*
     * Accumulated angular travel across the complete figure-eight phase.
     */
    float figure8_angle_traveled_ = 0.f;
};

/*
 * Global alias used by control-loop code that expects Waypoint at namespace
 * scope.
 */
using Waypoint =
BalanceTrajectoryController::Waypoint;

#endif
/**
 * @file    servo_test.c
 * @brief   Servo actuator test and characterization implementation.
 *
 * This module provides deterministic actuator tests above the
 * servo_actuator layer.
 *
 * The implementation contains:
 *
 *     - A predefined four-phase legacy actuator test.
 *     - Manual incremental servo adjustment.
 *     - Synchronized multi-servo sweep logging.
 *     - CSV logging of actuator commands and platform attitude.
 *
 * The test layer does not access PWM registers directly. All actuator
 * commands are routed through servo_actuator.
 */

#include "servo_test.h"

#include "servo_actuator.h"
#include "trajectory.h"
#include "system_state.h"
#include "calibration_data.h"

#include <stdio.h>

/*
 * Legacy test execution period.
 *
 * servo_test_step() is designed for a fixed 100 Hz update rate.
 * The newer servo_test_step_dt() interface accepts the actual control
 * loop period and therefore does not depend on this constant.
 */
#define TEST_DT   0.01f

/*
 * Trajectory limits used by the legacy trajectory-return test.
 *
 * The theoretical no-load servo speed is derived from the actuator
 * specification. The test deliberately uses a reduced velocity to
 * preserve margin under real mechanical load.
 *
 * Acceleration is an initial engineering estimate because the servo
 * specification does not provide a controlled acceleration value.
 * These values should therefore be characterized experimentally
 * before being treated as final actuator limits.
 */
#define SERVO_TEST_TRAJ_V_MAX   2800.0f
#define SERVO_TEST_TRAJ_A_MAX  20000.0f

typedef enum
{
    TEST_HOLD_NEUTRAL = 0,
    TEST_ABSOLUTE_SWEEP,
    TEST_INCREMENTAL,
    TEST_TRAJECTORY_HOME
} test_phase_t;

static test_phase_t s_phase = TEST_HOLD_NEUTRAL;
static uint32_t s_phase_tick = 0;

static trajectory_t s_traj[SERVO_CH_COUNT];

/**
 * @brief Return a human-readable name for a legacy test phase.
 *
 * The returned string is used only for diagnostic logging.
 */
static const char *test_phase_name(test_phase_t p)
{
    switch (p)
    {
        case TEST_HOLD_NEUTRAL:
            return "HOLD_NEUTRAL";

        case TEST_ABSOLUTE_SWEEP:
            return "ABSOLUTE_SWEEP";

        case TEST_INCREMENTAL:
            return "INCREMENTAL";

        case TEST_TRAJECTORY_HOME:
            return "TRAJECTORY_HOME";

        default:
            return "?";
    }
}

/**
 * @brief Initialize the legacy actuator test.
 */
void servo_test_init(void)
{
    servo_actuator_init();

    s_phase = TEST_HOLD_NEUTRAL;
    s_phase_tick = 0;
}

/**
 * @brief Execute one step of the legacy four-phase actuator test.
 *
 * The function assumes a fixed 100 Hz execution rate.
 *
 * Each phase exercises a different aspect of the actuator layer:
 *
 *     HOLD_NEUTRAL
 *         Startup stability at the neutral command.
 *
 *     ABSOLUTE_SWEEP
 *         Absolute target changes on S1 with actuator slew limiting.
 *
 *     INCREMENTAL
 *         Incremental target updates on S2 and boundary handling.
 *
 *     TRAJECTORY_HOME
 *         S3 is displaced and then returned using the trajectory
 *         generator.
 */
void servo_test_step(void)
{
    s_phase_tick++;

    switch (s_phase)
    {
        case TEST_HOLD_NEUTRAL:

            /*
             * Establish neutral commands at the beginning of the phase.
             * The following 2 seconds allow the platform to settle and
             * provide a stable reference for startup verification.
             */
            if (s_phase_tick == 1)
            {
                servo_actuator_set_target(
                    SERVO_CH_S1,
                    1500
                );

                servo_actuator_set_target(
                    SERVO_CH_S2,
                    1500
                );

                servo_actuator_set_target(
                    SERVO_CH_S3,
                    1500
                );
            }

            /*
             * 200 cycles at 10 ms correspond to 2 seconds.
             */
            if (s_phase_tick > 200)
            {
                s_phase = TEST_ABSOLUTE_SWEEP;
                s_phase_tick = 0;
            }

            break;

        case TEST_ABSOLUTE_SWEEP:

            /*
             * Exercise absolute target positioning on S1.
             *
             * The actuator layer is responsible for applying the
             * configured slew-rate limitation between these targets.
             */
            if (s_phase_tick == 1)
            {
                servo_actuator_set_target(
                    SERVO_CH_S1,
                    1000
                );
            }

            if (s_phase_tick == 300)
            {
                servo_actuator_set_target(
                    SERVO_CH_S1,
                    2000
                );
            }

            if (s_phase_tick == 600)
            {
                servo_actuator_set_target(
                    SERVO_CH_S1,
                    1500
                );
            }

            /*
             * Three 3-second moves complete this phase.
             */
            if (s_phase_tick > 900)
            {
                s_phase = TEST_INCREMENTAL;
                s_phase_tick = 0;
            }

            break;

        case TEST_INCREMENTAL:

            /*
             * Apply small incremental commands to S2.
             *
             * The actuator layer must enforce its configured limits,
             * preventing accumulated commands from exceeding the
             * valid actuator range.
             */
            if (s_phase_tick % 5 == 0)
            {
                servo_actuator_apply_delta(
                    SERVO_CH_S2,
                    +10
                );
            }

            /*
             * 400 cycles at 10 ms correspond to 4 seconds.
             */
            if (s_phase_tick > 400)
            {
                s_phase = TEST_TRAJECTORY_HOME;
                s_phase_tick = 0;
            }

            break;

        case TEST_TRAJECTORY_HOME:

            /*
             * Move S3 away from neutral before starting the trajectory.
             *
             * The actual actuator position is read back before starting
             * the trajectory so that the trajectory initial condition
             * matches the physical command state rather than assuming
             * a hard-coded starting position.
             */
            if (s_phase_tick == 1)
            {
                servo_actuator_set_target(
                    SERVO_CH_S3,
                    1000
                );
            }

            if (s_phase_tick == 100)
            {
                int32_t s1;
                int32_t s2;
                int32_t s3;

                servo_actuator_get_local(
                    &s1,
                    &s2,
                    &s3
                );

                trajectory_start(
                    &s_traj[SERVO_CH_S3],
                    (float)s3,
                    1500.0f,
                    SERVO_TEST_TRAJ_V_MAX,
                    SERVO_TEST_TRAJ_A_MAX
                );
            }

            if (s_phase_tick > 100)
            {
                /*
                 * Update the trajectory using the fixed legacy period.
                 *
                 * trajectory_update() provides the current position
                 * reference while velocity and acceleration outputs
                 * are not required by this test.
                 */
                float sp;

                trajectory_update(
                    &s_traj[SERVO_CH_S3],
                    TEST_DT,
                    &sp,
                    NULL,
                    NULL
                );

                servo_actuator_set_target(
                    SERVO_CH_S3,
                    (int32_t)sp
                );
            }

            /*
             * Allow sufficient time for the trajectory to complete
             * before restarting the test sequence.
             */
            if (s_phase_tick > 400)
            {
                s_phase = TEST_HOLD_NEUTRAL;
                s_phase_tick = 0;
            }

            break;
    }

    /*
     * Apply actuator slew-rate limiting and update the physical
     * actuator command once per test cycle.
     */
    servo_actuator_step(TEST_DT);

    /*
     * Emit a diagnostic status line every 0.5 seconds.
     *
     * This output is intentionally rate-limited so that debug logging
     * does not dominate the control-loop execution time.
     */
    if (s_phase_tick % 50 == 0)
    {
        int32_t s1;
        int32_t s2;
        int32_t s3;

        servo_actuator_get_local(
            &s1,
            &s2,
            &s3
        );

        printf(
            "[servo_test] phase=%-16s S1=%ld S2=%ld S3=%ld\r\n",
            test_phase_name(s_phase),
            (long)s1,
            (long)s2,
            (long)s3
        );
    }
}

/*
 * Synchronized sweep configuration.
 *
 * The sweep waits for the actuator to settle at each measurement point
 * before recording the corresponding servo and IMU values.
 */
#define SERVO_TEST_SWEEP_STEP_US       20
#define SERVO_TEST_SWEEP_SETTLE_CYCLES 10

static servo_test_mode_t s_mode = SERVO_TEST_MODE_LEGACY_B2;
static bool s_running = false;
static bool s_done = false;
static float s_elapsed_ms = 0.0f;

/*
 * List of servo channels included in the current test session.
 *
 * servo_ch = 0 selects S1, S2, and S3.
 * servo_ch = 1..3 selects one specific servo.
 */
static servo_ch_t s_ch_list[SERVO_CH_COUNT];
static uint8_t s_ch_count = 0;
static uint8_t s_ch_pos = 0;

typedef enum
{
    SWEEP_PHASE_SCAN = 0,
    SWEEP_PHASE_RETURN_NEUTRAL
} sweep_phase_t;

static sweep_phase_t s_sweep_phase = SWEEP_PHASE_SCAN;
static uint16_t s_sweep_settle_ct = 0;

/*
 * Sweep offset applied to the currently selected primary servo.
 *
 * Range:
 *
 *     -CALIB_TILT_MAX_US ... +CALIB_TILT_MAX_US
 *
 * The two non-primary servos compensate by -t/2 each.
 */
static float s_sweep_t = 0.0f;

/*
 * Servo selected for manual adjustment.
 *
 * Values are one-based to match the user-facing S1/S2/S3 numbering.
 */
static uint8_t s_manual_channel = 1;

/**
 * @brief Print the CSV header for actuator characterization data.
 */
void servo_test_log_csv_header(void)
{
    printf(
        "t_ms,S1_us,S2_us,S3_us,roll_deg,pitch_deg\r\n"
    );
}

/**
 * @brief Print one actuator and attitude measurement row.
 *
 * The actuator values represent the current local actuator state.
 * IMU values are read from the system state interface.
 */
void servo_test_log_csv_row(uint32_t t_ms)
{
    int32_t s1;
    int32_t s2;
    int32_t s3;

    float roll;
    float pitch;
    float vroll;
    float vpitch;

    servo_actuator_get_local(
        &s1,
        &s2,
        &s3
    );

    imu_state_read(
        system_state_get_imu_ptr(),
        &roll,
        &pitch,
        &vroll,
        &vpitch
    );

    printf(
        "%lu,%ld,%ld,%ld,%.2f,%.2f\r\n",
        (unsigned long)t_ms,
        (long)s1,
        (long)s2,
        (long)s3,
        (double)roll,
        (double)pitch
    );
}

/**
 * @brief Build the active servo-channel list for a test session.
 *
 * @param servo_ch 0 selects all channels; 1..3 selects one channel.
 */
static void build_channel_list(uint8_t servo_ch)
{
    if (servo_ch == 0)
    {
        s_ch_list[0] = SERVO_CH_S1;
        s_ch_list[1] = SERVO_CH_S2;
        s_ch_list[2] = SERVO_CH_S3;

        s_ch_count = 3;
    }
    else
    {
        s_ch_list[0] =
            (servo_ch_t)(servo_ch - 1);

        s_ch_count = 1;
    }

    s_ch_pos = 0;
}

/**
 * @brief Start a new controlled servo test session.
 *
 * The session state is reset before configuring the selected test
 * mode. The initial sweep offset is set to the negative calibration
 * limit so that every sweep begins from a deterministic condition.
 */
void servo_test_start(
    servo_test_mode_t mode,
    uint8_t servo_ch
)
{
    s_mode = mode;
    s_running = true;
    s_done = false;
    s_elapsed_ms = 0.0f;

    build_channel_list(servo_ch);

    s_sweep_phase = SWEEP_PHASE_SCAN;
    s_sweep_settle_ct = 0;
    s_sweep_t = -(float)CALIB_TILT_MAX_US;

    /*
     * Manual mode uses the explicitly selected channel.
     * If no valid individual channel is supplied, default to S1.
     */
    s_manual_channel =
        (servo_ch >= 1 && servo_ch <= 3)
            ? servo_ch
            : 1;

    servo_test_log_csv_header();
}

/**
 * @brief Stop the current controlled test session.
 */
void servo_test_stop(void)
{
    s_running = false;
}

/**
 * @brief Return the completion state of the current test session.
 */
bool servo_test_is_done(void)
{
    return s_done;
}

/**
 * @brief Select the active channel for manual adjustment.
 *
 * Invalid channel numbers are ignored so that an invalid UI command
 * cannot select an undefined actuator.
 */
void servo_test_manual_select_channel(uint8_t servo_ch)
{
    if (servo_ch >= 1 && servo_ch <= 3)
    {
        s_manual_channel = servo_ch;
    }
}

/**
 * @brief Apply an incremental command in manual-step mode.
 *
 * The actuator layer performs the actual position update and applies
 * its configured limits and slew-rate behavior.
 *
 * A CSV measurement is recorded immediately after the command is
 * issued. Therefore the logged servo position represents the current
 * actuator state, not necessarily the final target position.
 */
void servo_test_manual_adjust(int16_t delta_us)
{
    servo_ch_t ch =
        (servo_ch_t)(s_manual_channel - 1);

    servo_actuator_apply_delta(
        ch,
        delta_us
    );

    servo_test_log_csv_row(
        (uint32_t)s_elapsed_ms
    );
}

/**
 * @brief Execute one synchronized servo sweep step.
 *
 * The currently selected servo acts as the primary axis:
 *
 *     primary = neutral + t
 *
 * The other two servos compensate equally:
 *
 *     secondary = neutral - t/2
 *
 * Therefore:
 *
 *     ΔS1 + ΔS2 + ΔS3 = 0
 *
 * for every sweep position t.
 *
 * This preserves the mechanical zero-sum constraint throughout the
 * sweep rather than only at the endpoints.
 */
static void sweep_log_step(void)
{
    if (s_ch_pos >= s_ch_count)
    {
        s_done = true;
        s_running = false;
        return;
    }

    servo_ch_t primary =
        s_ch_list[s_ch_pos];

    const calibration_data_t *c =
        calibration_data_get_ptr();

    int32_t neutral[3] =
    {
        c->S1_neutral,
        c->S2_neutral,
        c->S3_neutral
    };

    switch (s_sweep_phase)
    {
        case SWEEP_PHASE_SCAN:

            /*
             * Issue a new target only at the beginning of each
             * settle interval.
             */
            if (s_sweep_settle_ct == 0)
            {
                int32_t tgt[3];

                /*
                 * Apply the zero-sum mechanical constraint:
                 *
                 *     t + (-t/2) + (-t/2) = 0
                 *
                 * for every value of t.
                 */
                for (uint8_t i = 0; i < 3; i++)
                {
                    tgt[i] =
                        (i == primary)
                            ? (
                                neutral[i] +
                                (int32_t)s_sweep_t
                              )
                            : (
                                neutral[i] -
                                (int32_t)(
                                    s_sweep_t / 2.0f
                                )
                              );
                }

                servo_actuator_set_target(
                    SERVO_CH_S1,
                    tgt[0]
                );

                servo_actuator_set_target(
                    SERVO_CH_S2,
                    tgt[1]
                );

                servo_actuator_set_target(
                    SERVO_CH_S3,
                    tgt[2]
                );
            }

            s_sweep_settle_ct++;

            /*
             * Record a measurement only after the configured
             * settling interval has elapsed.
             */
            if (
                s_sweep_settle_ct >=
                SERVO_TEST_SWEEP_SETTLE_CYCLES
            )
            {
                servo_test_log_csv_row(
                    (uint32_t)s_elapsed_ms
                );

                s_sweep_settle_ct = 0;

                if (
                    s_sweep_t >=
                    (float)CALIB_TILT_MAX_US
                )
                {
                    s_sweep_phase =
                        SWEEP_PHASE_RETURN_NEUTRAL;
                }
                else
                {
                    s_sweep_t +=
                        (float)SERVO_TEST_SWEEP_STEP_US;

                    /*
                     * Clamp the final point to the exact calibration
                     * boundary when the sweep step does not divide
                     * the range evenly.
                     */
                    if (
                        s_sweep_t >
                        (float)CALIB_TILT_MAX_US
                    )
                    {
                        s_sweep_t =
                            (float)CALIB_TILT_MAX_US;
                    }
                }
            }

            break;

        case SWEEP_PHASE_RETURN_NEUTRAL:

            /*
             * Restore all servos to their calibrated neutral positions
             * before starting the next primary-axis sweep.
             */
            if (s_sweep_settle_ct == 0)
            {
                servo_actuator_set_target(
                    SERVO_CH_S1,
                    neutral[0]
                );

                servo_actuator_set_target(
                    SERVO_CH_S2,
                    neutral[1]
                );

                servo_actuator_set_target(
                    SERVO_CH_S3,
                    neutral[2]
                );
            }

            s_sweep_settle_ct++;

            if (
                s_sweep_settle_ct >=
                SERVO_TEST_SWEEP_SETTLE_CYCLES
            )
            {
                s_sweep_settle_ct = 0;

                s_sweep_phase =
                    SWEEP_PHASE_SCAN;

                /*
                 * Reset the sweep offset for the next primary servo.
                 */
                s_sweep_t =
                    -(float)CALIB_TILT_MAX_US;

                s_ch_pos++;
            }

            break;
    }
}

/**
 * @brief Execute one controlled test-loop update.
 *
 * The actual elapsed time is supplied by the caller. This allows
 * manual and sweep tests to run correctly inside the real control
 * loop without assuming a fixed task frequency.
 */
void servo_test_step_dt(float dt)
{
    if (!s_running)
    {
        return;
    }

    s_elapsed_ms += dt * 1000.0f;

    switch (s_mode)
    {
        case SERVO_TEST_MODE_MANUAL_STEP:

            /*
             * Manual mode changes actuator targets only in response
             * to explicit servo_test_manual_adjust() commands.
             */
            break;

        case SERVO_TEST_MODE_SWEEP_LOG:

            sweep_log_step();
            break;

        default:
            break;
    }

    /*
     * Apply actuator slew-rate limiting and update the physical
     * actuator command once per control-loop cycle.
     */
    servo_actuator_step(dt);
}

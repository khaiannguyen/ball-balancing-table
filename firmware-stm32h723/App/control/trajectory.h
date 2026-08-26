/**
 * @file    trajectory.h
 * @brief   Trajectory generation and acceleration-continuous replanning.
 *
 * The trajectory engine represents motion as a sequence of constant-
 * acceleration segments. A trajectory can be replanned from its current
 * reference position and velocity without forcing the velocity back to zero.
 *
 * This preserves velocity continuity when a new target arrives while an
 * actuator is already moving.
 *
 * The current implementation is acceleration-limited. The a0 parameter is
 * retained in the replan API for future jerk-limited trajectory generation.
 *
 * The module is independent of HAL and RTOS and can therefore be unit-tested
 * separately from the embedded application.
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /*
     * Stop motion before reversing direction toward a new target.
     */
    TRAJ_PHASE_BRAKE = 0,

    /*
     * Increase velocity toward the selected peak velocity.
     */
    TRAJ_PHASE_ACCEL,

    /*
     * Maintain the selected peak velocity at constant speed.
     */
    TRAJ_PHASE_CRUISE,

    /*
     * Reduce velocity to zero at the target.
     */
    TRAJ_PHASE_DECEL,
} trajectory_phase_t;

/*
 * One constant-acceleration trajectory segment.
 *
 * x_start and v_start describe the reference state at the beginning of
 * this segment. The segment is evaluated using:
 *
 *     x(t) = x_start + v_start * t + 0.5 * a_const * t^2
 *     v(t) = v_start + a_const * t
 *
 * where t is measured from the beginning of this segment.
 */
typedef struct {
    trajectory_phase_t type;
    float duration;
    float x_start;
    float v_start;
    float a_const;
} trajectory_segment_t;

/**
 * @brief Complete runtime state of one trajectory axis.
 *
 * All trajectory segments are generated during trajectory_replan().
 * trajectory_update() then advances through those precomputed segments
 * according to dt.
 *
 * The x/v/a outputs represent the latest trajectory reference state.
 * They are command-side references, not measured actuator positions.
 */
typedef struct {
    /*
     * Maximum sequence:
     * BRAKE -> ACCEL -> CRUISE -> DECEL.
     */
    trajectory_segment_t seg[4];

    int seg_count;

    /*
     * Index of the currently active segment.
     * seg_index == seg_count indicates completion.
     */
    int seg_index;

    /*
     * Elapsed time within the active segment.
     */
    float t_in_seg;

    float target;
    float v_max;
    float a_max;

    /*
     * Latest trajectory reference state.
     *
     * These values are used as x0/v0/a0 when the next trajectory is
     * replanned. They describe the commanded trajectory state and are
     * not feedback from the physical servo.
     */
    float x;
    float v;
    float a;

} trajectory_state_t;

/**
 * @brief Replan a trajectory from the current reference state.
 *
 * The current velocity is preserved instead of being reset to zero.
 *
 * If the current velocity points away from the new target, an explicit
 * BRAKE phase is inserted first. Once the reference velocity is aligned
 * with the target direction, the remaining motion is generated as
 * ACCEL/CRUISE/DECEL.
 *
 * @param tr      Trajectory state to overwrite completely.
 * @param x0      Current trajectory reference position.
 * @param v0      Current trajectory reference velocity.
 * @param a0      Current acceleration. Reserved for future jerk-limited
 *                trajectory generation.
 * @param target  New target position.
 * @param v_max   Maximum velocity in position units per second.
 * @param a_max   Maximum acceleration in position units per second squared.
 */
void trajectory_replan(
    trajectory_state_t *tr,
    float x0,
    float v0,
    float a0,
    float target,
    float v_max,
    float a_max
);

/**
 * @brief Advance the trajectory by one control cycle.
 *
 * The function may cross multiple segments when dt is larger than the
 * remaining duration of the current segment. Once all segments are complete,
 * the reference remains exactly at (target, 0, 0) on subsequent calls.
 *
 * @param tr     Trajectory state previously initialized by trajectory_replan().
 * @param dt     Elapsed control-loop time in seconds.
 * @param x_ref  Output position reference. Pass NULL if not required.
 * @param v_ref  Output velocity reference. Pass NULL if not required.
 * @param a_ref  Output acceleration reference. Pass NULL if not required.
 */
void trajectory_update(
    trajectory_state_t *tr,
    float dt,
    float *x_ref,
    float *v_ref,
    float *a_ref
);

/**
 * @brief Check whether a trajectory has completed all segments.
 *
 * @param tr Trajectory state.
 *
 * @return true when the trajectory has reached its target.
 */
static inline bool trajectory_state_is_done(
    const trajectory_state_t *tr)
{
    return tr->seg_index >= tr->seg_count;
}

/*
 * Compatibility API for trajectories that always start from rest.
 *
 * These wrappers are intended for Home, Calibration, and other operations
 * where the trajectory intentionally starts with v0 = 0 and a0 = 0.
 *
 * Ball control should use trajectory_replan() and trajectory_update()
 * directly so velocity continuity is preserved when setpoints change.
 */
typedef trajectory_state_t trajectory_t;

/**
 * @brief Start a trajectory from a stationary state.
 *
 * Equivalent to trajectory_replan() with v0 = 0 and a0 = 0.
 *
 * @param tr     Trajectory state to initialize.
 * @param from   Starting position.
 * @param to     Target position.
 * @param v_max  Maximum velocity.
 * @param a_max  Maximum acceleration.
 */
void trajectory_start(
    trajectory_t *tr,
    float from,
    float to,
    float v_max,
    float a_max
);

/**
 * @brief Check whether a compatibility trajectory has completed.
 *
 * @param tr Trajectory state.
 *
 * @return true when the trajectory has reached its target.
 */
static inline bool trajectory_is_done(const trajectory_t *tr)
{
    return trajectory_state_is_done(tr);
}

/**
 * @brief Compute the natural trajectory duration.
 *
 * Generates a temporary trajectory using the specified velocity and
 * acceleration limits and returns the sum of all segment durations.
 *
 * This function does not modify any caller-owned trajectory state.
 *
 * @param from   Starting position.
 * @param to     Target position.
 * @param v_max  Maximum velocity.
 * @param a_max  Maximum acceleration.
 *
 * @return Natural trajectory duration in seconds.
 *         Returns 0.0f when no motion is required.
 */
float trajectory_compute_time(
    float from,
    float to,
    float v_max,
    float a_max
);

/**
 * @brief Start a trajectory with an optional longer completion time.
 *
 * If target_time is greater than the natural trajectory duration, the
 * velocity and acceleration limits are scaled down so the same motion
 * completes at target_time.
 *
 * The function only stretches the trajectory. It never increases the
 * configured velocity or acceleration limits to force a shorter motion.
 *
 * @param tr          Trajectory state to initialize.
 * @param from        Starting position.
 * @param to          Target position.
 * @param v_max       Maximum velocity before time scaling.
 * @param a_max       Maximum acceleration before time scaling.
 * @param target_time Requested completion time in seconds.
 */
void trajectory_start_scaled(
    trajectory_t *tr,
    float from,
    float to,
    float v_max,
    float a_max,
    float target_time
);

/**
 * @brief Generate synchronized trajectories for three axes.
 *
 * Each axis is first evaluated using the same velocity and acceleration
 * limits. The axis with the longest natural duration defines the common
 * completion time. Faster axes are then time-scaled so all three targets
 * are reached simultaneously.
 *
 * @param tr     Array of three trajectory states.
 * @param from   Starting positions for S1, S2, and S3.
 * @param to     Target positions for S1, S2, and S3.
 * @param v_max  Common maximum velocity before scaling.
 * @param a_max  Common maximum acceleration before scaling.
 */
void trajectory_start_synced3(
    trajectory_t tr[3],
    const float from[3],
    const float to[3],
    float v_max,
    float a_max
);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */

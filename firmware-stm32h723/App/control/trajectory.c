/**
 * @file    trajectory.c
 * @brief   Trajectory generation and acceleration-continuous replanning.
 *
 * The trajectory engine generates piecewise-constant-acceleration motion
 * profiles and supports replanning from the current reference state without
 * resetting velocity to zero.
 *
 * The implementation is independent of HAL and RTOS so that the trajectory
 * logic can be unit-tested separately from the embedded application.
 */

#include "trajectory.h"
#include <math.h>
#include <string.h>

/*
 * Floating-point tolerances used to avoid direct equality comparisons.
 * The values are intentionally small relative to the position and velocity
 * units used by the caller.
 */
#define TRAJ_X_EPS   1e-4f
#define TRAJ_V_EPS   1e-4f

/*
 * Evaluate one constant-acceleration segment.
 *
 * The segment state is described by its initial position, velocity, and
 * constant acceleration. Time is measured from the beginning of the segment.
 */
static void eval_const_accel(float x0, float v0, float a, float t,
                            float *x, float *v)
{
    *x = x0 + v0 * t + 0.5f * a * t * t;
    *v = v0 + a * t;
}

/*
 * Build the ACCEL/CRUISE/DECEL portion of a trajectory.
 *
 * The initial velocity must already be aligned with the requested direction.
 * If the remaining distance is too short to reach v_max, the generated
 * profile automatically becomes triangular by omitting the cruise phase.
 *
 * The peak velocity is derived from the remaining distance and current
 * velocity so that the profile reaches the target with zero velocity.
 */
static int build_ramp_segments(trajectory_state_t *tr, int base_idx,
                               float x_start, float v_signed, float target,
                               float dir, float v_max, float a_max)
{
    float d = (target - x_start) * dir;

    if (d < TRAJ_X_EPS) {
        return 0;
    }

    /*
     * Express the current velocity along the direction of travel.
     * Negative projection should already have been handled by the brake phase.
     */
    float s = v_signed * dir;

    if (s < 0.0f) {
        s = 0.0f;
    }

    /*
     * Keep the current velocity inside the active velocity limit.
     * This also protects against a v_max reduction while the trajectory
     * is being replanned.
     */
    if (s > v_max) {
        s = v_max;
    }

    /*
     * Compute the peak velocity required to accelerate from the current
     * speed and decelerate to zero over the remaining distance.
     *
     * If the remaining distance is too short for a full deceleration phase,
     * preserve the current speed as the limiting peak speed.
     */
    float v_peak_sq = a_max * d + 0.5f * s * s;

    if (v_peak_sq < s * s) {
        v_peak_sq = s * s;
    }

    float v_peak = sqrtf(v_peak_sq);

    if (v_peak > v_max) {
        v_peak = v_max;
    }

    float a_signed = a_max * dir;
    float t_accel  = (v_peak > s) ? (v_peak - s) / a_max : 0.0f;
    float t_decel  = v_peak / a_max;

    int   idx   = base_idx;
    float seg_x = x_start;
    float seg_v = v_signed;

    if (t_accel > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_ACCEL;
        tr->seg[idx].duration = t_accel;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;
        tr->seg[idx].a_const  = a_signed;

        float xe, ve;

        eval_const_accel(
            seg_x,
            seg_v,
            a_signed,
            t_accel,
            &xe,
            &ve
        );

        seg_x = xe;
        seg_v = ve;
        idx++;
    }

    float d_so_far = (seg_x - x_start) * dir;
    float d_decel  = (v_peak * v_peak) / (2.0f * a_max);
    float d_cruise = d - d_so_far - d_decel;

    if (d_cruise < 0.0f) {
        d_cruise = 0.0f;
    }

    float t_cruise =
        (v_peak > TRAJ_V_EPS) ? (d_cruise / v_peak) : 0.0f;

    if (t_cruise > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_CRUISE;
        tr->seg[idx].duration = t_cruise;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;
        tr->seg[idx].a_const  = 0.0f;

        float xe, ve;

        eval_const_accel(
            seg_x,
            seg_v,
            0.0f,
            t_cruise,
            &xe,
            &ve
        );

        seg_x = xe;
        seg_v = ve;
        idx++;
    }

    if (t_decel > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_DECEL;
        tr->seg[idx].duration = t_decel;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;

        /*
         * Deceleration uses acceleration opposite to the direction of travel
         * so that velocity reaches zero at the target.
         */
        tr->seg[idx].a_const = -a_signed;
        idx++;
    }

    return idx - base_idx;
}

/*
 * Build an explicit braking phase when the current velocity points away
 * from the requested target.
 *
 * The brake phase brings the velocity to zero before the normal
 * ACCEL/CRUISE/DECEL profile is generated.
 */
static int build_brake_segment(trajectory_state_t *tr, int base_idx,
                               float x_start, float v0, float a_max)
{
    if (fabsf(v0) < TRAJ_V_EPS) {
        return 0;
    }

    float dir0 = (v0 > 0.0f) ? 1.0f : -1.0f;
    float t_brake = fabsf(v0) / a_max;

    if (t_brake <= TRAJ_V_EPS) {
        return 0;
    }

    tr->seg[base_idx].type     = TRAJ_PHASE_BRAKE;
    tr->seg[base_idx].duration = t_brake;
    tr->seg[base_idx].x_start  = x_start;
    tr->seg[base_idx].v_start  = v0;

    /*
     * Apply acceleration opposite to the current velocity so that the
     * actuator stops before changing direction toward the new target.
     */
    tr->seg[base_idx].a_const = -a_max * dir0;

    return 1;
}

void trajectory_replan(trajectory_state_t *tr, float x0, float v0, float a0,
                       float target, float v_max, float a_max)
{
    /*
     * a0 is part of the public state interface for future jerk-limited
     * trajectory generation. The current profile uses constant acceleration
     * within each segment and therefore does not require the incoming a0.
     */
    (void)a0;

    memset(tr, 0, sizeof(*tr));

    tr->target = target;
    tr->v_max  = v_max;
    tr->a_max  = a_max;
    tr->x      = x0;
    tr->v      = v0;
    tr->a      = 0.0f;

    float dx = target - x0;

    /*
     * If the target is already reached and the actuator is stationary,
     * no trajectory segments are required.
     */
    if (fabsf(dx) < TRAJ_X_EPS && fabsf(v0) < TRAJ_V_EPS) {
        tr->seg_count = 0;
        tr->seg_index = 0;
        return;
    }

    /*
     * Select the direction toward the target.
     * When position error is negligible, use the current velocity direction
     * so that an active trajectory can still be completed correctly.
     */
    float dir =
        (fabsf(dx) < TRAJ_X_EPS)
            ? ((v0 > 0.0f) ? 1.0f : -1.0f)
            : ((dx > 0.0f) ? 1.0f : -1.0f);

    int   n  = 0;
    float cx = x0;
    float cv = v0;

    if (v0 * dir < -TRAJ_V_EPS) {
        /*
         * The actuator is currently moving away from the new target.
         * Brake first so the following trajectory starts from zero velocity
         * in the correct direction.
         */
        int added = build_brake_segment(
            tr,
            n,
            cx,
            v0,
            a_max
        );

        if (added > 0) {
            trajectory_segment_t *b = &tr->seg[n];
            float xe, ve;

            eval_const_accel(
                b->x_start,
                b->v_start,
                b->a_const,
                b->duration,
                &xe,
                &ve
            );

            cx = xe;
            cv = ve;
            n += added;
        }

        /*
         * Recalculate the target direction after braking because the actuator
         * may have crossed the target during the braking phase.
         */
        dx = target - cx;

        if (fabsf(dx) < TRAJ_X_EPS) {
            tr->seg_count = n;
            tr->seg_index = 0;
            return;
        }

        dir = (dx > 0.0f) ? 1.0f : -1.0f;
    }

    /*
     * The current velocity is now aligned with the target direction.
     * Generate the remaining ACCEL/CRUISE/DECEL profile from the actual
     * reference state instead of restarting from zero velocity.
     */
    n += build_ramp_segments(
        tr,
        n,
        cx,
        cv,
        target,
        dir,
        v_max,
        a_max
    );

    tr->seg_count = n;
    tr->seg_index = 0;
}

void trajectory_update(trajectory_state_t *tr, float dt,
                       float *x_ref, float *v_ref, float *a_ref)
{
    /*
     * A negative control period is invalid. Clamp it to zero rather than
     * moving the trajectory backward in time.
     */
    if (dt < 0.0f) {
        dt = 0.0f;
    }

    float dt_left = dt;

    /*
     * Advance through as many segments as necessary.
     *
     * The loop intentionally handles dt values larger than one segment
     * duration so no trajectory phase is skipped during a delayed cycle.
     */
    while (tr->seg_index < tr->seg_count && dt_left > 0.0f) {
        trajectory_segment_t *seg = &tr->seg[tr->seg_index];

        float remain = seg->duration - tr->t_in_seg;

        if (remain < 0.0f) {
            remain = 0.0f;
        }

        if (dt_left < remain) {
            tr->t_in_seg += dt_left;
            dt_left = 0.0f;
        } else {
            dt_left -= remain;
            tr->seg_index++;
            tr->t_in_seg = 0.0f;
        }
    }

    if (tr->seg_index >= tr->seg_count) {
        /*
         * Clamp the final reference exactly to the target.
         * Do not extrapolate beyond the final segment because accumulated
         * floating-point error could otherwise produce a small overshoot.
         */
        tr->x = tr->target;
        tr->v = 0.0f;
        tr->a = 0.0f;
    } else {
        trajectory_segment_t *seg = &tr->seg[tr->seg_index];

        float x, v;

        eval_const_accel(
            seg->x_start,
            seg->v_start,
            seg->a_const,
            tr->t_in_seg,
            &x,
            &v
        );

        tr->x = x;
        tr->v = v;
        tr->a = seg->a_const;
    }

    if (x_ref) {
        *x_ref = tr->x;
    }

    if (v_ref) {
        *v_ref = tr->v;
    }

    if (a_ref) {
        *a_ref = tr->a;
    }
}

/*
 * Compatibility wrapper for callers that always start from rest.
 *
 * Home and Calibration use this API because their trajectories intentionally
 * begin from a stationary reference. Ball control uses trajectory_replan()
 * directly so velocity continuity is preserved during setpoint changes.
 */
void trajectory_start(trajectory_t *tr, float from, float to,
                      float v_max, float a_max)
{
    trajectory_replan(
        tr,
        from,
        0.0f,
        0.0f,
        to,
        v_max,
        a_max
    );
}

float trajectory_compute_time(float from, float to,
                              float v_max, float a_max)
{
    trajectory_state_t tmp;

    trajectory_replan(
        &tmp,
        from,
        0.0f,
        0.0f,
        to,
        v_max,
        a_max
    );

    float total = 0.0f;

    for (int i = 0; i < tmp.seg_count; i++) {
        total += tmp.seg[i].duration;
    }

    return total;
}

void trajectory_start_scaled(trajectory_t *tr, float from, float to,
                             float v_max, float a_max, float target_time)
{
    float t0 = trajectory_compute_time(
        from,
        to,
        v_max,
        a_max
    );

    /*
     * Only stretch the natural trajectory duration.
     *
     * If the requested duration is shorter than the physical trajectory,
     * keep the original limits instead of increasing them beyond the
     * configured actuator capability.
     */
    if (t0 <= 1e-6f || target_time <= t0) {
        trajectory_start(
            tr,
            from,
            to,
            v_max,
            a_max
        );
        return;
    }

    float s = target_time / t0;

    /*
     * Position scaling requires velocity to scale by 1/s and acceleration
     * by 1/s^2 so the trajectory reaches the same target in the requested
     * longer duration without exceeding the original limits.
     */
    trajectory_start(
        tr,
        from,
        to,
        v_max / s,
        a_max / (s * s)
    );
}

void trajectory_start_synced3(trajectory_t tr[3], const float from[3],
                              const float to[3], float v_max, float a_max)
{
    float t0[3];

    for (int i = 0; i < 3; i++) {
        t0[i] = trajectory_compute_time(
            from[i],
            to[i],
            v_max,
            a_max
        );
    }

    /*
     * Use the slowest axis as the common completion time.
     * The remaining axes are time-scaled so all three targets are reached
     * simultaneously without increasing the configured motion limits.
     */
    float T = t0[0];

    if (t0[1] > T) {
        T = t0[1];
    }

    if (t0[2] > T) {
        T = t0[2];
    }

    for (int i = 0; i < 3; i++) {
        trajectory_start_scaled(
            &tr[i],
            from[i],
            to[i],
            v_max,
            a_max,
            T
        );
    }
}

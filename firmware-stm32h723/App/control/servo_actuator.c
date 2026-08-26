/**
 * @file    servo_actuator.c
 * @brief   Actuator layer implementation.
 *
 * Provides the final motion-limiting layer before the physical PWM output.
 * The module supports position, velocity, and incremental commands while
 * enforcing actuator limits and publishing the current actuator state.
 *
 * @see servo_actuator.h
 */

#include "servo_actuator.h"
#include "system_state.h"
#include <stddef.h>

/*
 * Actuator state is owned by system_state.c and exposed through an accessor.
 * Keeping the state ownership in one module allows other tasks to consume
 * a consistent snapshot without accessing the internal actuator state.
 */
static actuator_state_t *s_actuator_state = NULL;

typedef struct {
    int32_t neutral;
    int32_t min;
    int32_t max;
    int32_t deadband_us;
} servo_axis_calib_t;

/*
 * Default actuator limits used until valid calibration data is loaded.
 * These values provide a bounded operating range for initial bring-up.
 */
static servo_axis_calib_t s_calib[SERVO_CH_COUNT] = {
    [SERVO_CH_S1] = {
        .neutral = 1500,
        .min = 1000,
        .max = 2000,
        .deadband_us = 0
    },
    [SERVO_CH_S2] = {
        .neutral = 1500,
        .min = 1000,
        .max = 2000,
        .deadband_us = 0
    },
    [SERVO_CH_S3] = {
        .neutral = 1500,
        .min = 1000,
        .max = 2000,
        .deadband_us = 0
    },
};

typedef struct {
    int32_t pos_us;
    int32_t target_us;
    int32_t last_dir;
    float   velocity_us_s;
    float   vel_current_us_s;
    bool    speed_mode;
} servo_axis_state_t;

/*
 * pos_us is the last actuator position actually applied.
 * target_us is the requested position used by position/incremental mode.
 * vel_current_us_s is the velocity after acceleration limiting and therefore
 * represents the motion actually applied during the current control cycle.
 */
static servo_axis_state_t s_axis[SERVO_CH_COUNT];

void servo_actuator_init(void)
{
    for (int ch = 0; ch < SERVO_CH_COUNT; ch++) {
        s_axis[ch].pos_us           = s_calib[ch].neutral;
        s_axis[ch].target_us        = s_calib[ch].neutral;
        s_axis[ch].last_dir         = 0;
        s_axis[ch].velocity_us_s    = 0.0f;
        s_axis[ch].vel_current_us_s = 0.0f;
        s_axis[ch].speed_mode       = false;
    }

    s_actuator_state = system_state_get_actuator_ptr();

    /*
     * Initialize the physical PWM outputs at the calibrated neutral position.
     */
    servo_pwm_init();
}

void servo_actuator_set_target(servo_ch_t ch, int32_t target_us)
{
    if (ch >= SERVO_CH_COUNT) {
        return;
    }

    /*
     * A position command always leaves velocity mode.
     * Range limiting is deferred to servo_actuator_step() so that all
     * actuator constraints are applied in one place.
     */
    s_axis[ch].speed_mode = false;
    s_axis[ch].target_us  = target_us;
}

void servo_actuator_set_velocity(servo_ch_t ch, float us_per_sec)
{
    if (ch >= SERVO_CH_COUNT) {
        return;
    }

    if (us_per_sec == 0.0f) {
        /*
         * Stop at the current physical position instead of changing the
         * target. This avoids an additional motion command when velocity
         * mode is disabled.
         */
        s_axis[ch].speed_mode = false;
        s_axis[ch].target_us  = s_axis[ch].pos_us;
    } else {
        s_axis[ch].speed_mode    = true;
        s_axis[ch].velocity_us_s = us_per_sec;
    }
}

void servo_actuator_apply_delta(servo_ch_t ch, int32_t delta_us)
{
    if (ch >= SERVO_CH_COUNT) {
        return;
    }

    /*
     * Incremental commands accumulate on the current target.
     * Clamp, slew-rate, and acceleration limits are applied later by
     * servo_actuator_step().
     */
    s_axis[ch].speed_mode = false;
    s_axis[ch].target_us += delta_us;
}

void servo_actuator_get_local(int32_t *s1, int32_t *s2, int32_t *s3)
{
    if (s1) {
        *s1 = s_axis[SERVO_CH_S1].pos_us;
    }

    if (s2) {
        *s2 = s_axis[SERVO_CH_S2].pos_us;
    }

    if (s3) {
        *s3 = s_axis[SERVO_CH_S3].pos_us;
    }
}

void servo_actuator_set_calib(
    servo_ch_t ch,
    int32_t neutral,
    int32_t min,
    int32_t max,
    int32_t deadband_us)
{
    if (ch >= SERVO_CH_COUNT) {
        return;
    }

    s_calib[ch].neutral     = neutral;
    s_calib[ch].min         = min;
    s_calib[ch].max         = max;
    s_calib[ch].deadband_us = deadband_us;
}

void servo_actuator_step(float dt)
{
    /*
     * A valid positive dt is required to convert position error into
     * a velocity command and to apply the acceleration limit.
     */
    if (dt <= 0.0f) {
        return;
    }

    for (int ch = 0; ch < SERVO_CH_COUNT; ch++) {
        servo_axis_state_t *a = &s_axis[ch];
        servo_axis_calib_t *c = &s_calib[ch];

        /*
         * Convert the active command into the velocity requested for
         * this control cycle.
         *
         * Velocity mode uses the commanded velocity directly.
         * Position mode derives the velocity required to reach the target
         * within the current control period.
         */
        float desired_v;

        if (a->speed_mode) {
            desired_v = a->velocity_us_s;
        } else {
            desired_v = (float)(a->target_us - a->pos_us) / dt;
        }

        /*
         * Prevent further motion when the physical calibration limit has
         * already been reached in the commanded direction.
         *
         * This prevents the velocity command from continuing to accumulate
         * while the actuator is constrained by the position limit.
         */
        if ((a->pos_us >= c->max && desired_v > 0.0f) ||
            (a->pos_us <= c->min && desired_v < 0.0f)) {
            desired_v = 0.0f;
        }

        /*
         * Apply the actuator slew-rate limit.
         *
         * This is a hard velocity boundary shared by all command sources.
         * Higher-level trajectory generation remains responsible for
         * producing the intended motion profile.
         */
        if (desired_v > SERVO_SLEW_MAX_US_PER_S) {
            desired_v = SERVO_SLEW_MAX_US_PER_S;
        }

        if (desired_v < -SERVO_SLEW_MAX_US_PER_S) {
            desired_v = -SERVO_SLEW_MAX_US_PER_S;
        }

        /*
         * Limit how quickly the applied velocity can change.
         *
         * vel_current_us_s represents the velocity actually applied to
         * the actuator. This layer therefore provides the final acceleration
         * boundary before the PWM command.
         */
        float max_dv = SERVO_ACCEL_MAX_US_PER_S2 * dt;
        float dv = desired_v - a->vel_current_us_s;

        if (dv > max_dv) {
            dv = max_dv;
        }

        if (dv < -max_dv) {
            dv = -max_dv;
        }

        a->vel_current_us_s += dv;

        int32_t delta = (int32_t)(a->vel_current_us_s * dt);

        /*
         * Apply deadband compensation when the commanded direction changes.
         * The compensation offsets mechanical backlash at the actuator
         * without changing the direction of the higher-level trajectory.
         */
        int32_t dir = (delta > 0) - (delta < 0);

        if (dir != 0 &&
            dir != a->last_dir &&
            c->deadband_us > 0) {
            delta += dir * c->deadband_us;
        }

        if (dir != 0) {
            a->last_dir = dir;
        }

        /*
         * Update the physical position and enforce the calibrated hard
         * limits after applying the motion increment.
         */
        a->pos_us += delta;

        if (a->pos_us < c->min) {
            a->pos_us = c->min;
        }

        if (a->pos_us > c->max) {
            a->pos_us = c->max;
        }

        /*
         * Keep the position target inside the same calibrated range.
         * This prevents repeated incremental commands from accumulating
         * an unreachable target outside the actuator limits.
         */
        if (!a->speed_mode) {
            if (a->target_us < c->min) {
                a->target_us = c->min;
            }

            if (a->target_us > c->max) {
                a->target_us = c->max;
            }
        }

        /*
         * Convert the constrained internal position into the physical PWM
         * command. This is the final actuator output stage.
         */
        servo_pwm_write_us(
            (servo_ch_t)ch,
            (uint16_t)a->pos_us
        );
    }

    /*
     * Publish the completed actuator snapshot after all channels have been
     * updated. Other tasks consume this state through the synchronized
     * system-state interface rather than accessing s_axis directly.
     */
    if (s_actuator_state != NULL) {
        actuator_state_publish(
            s_actuator_state,
            s_axis[SERVO_CH_S1].pos_us,
            s_axis[SERVO_CH_S2].pos_us,
            s_axis[SERVO_CH_S3].pos_us
        );
    }
}

/**
 * @file    servo_actuator.h
 * @brief   Final actuator limiting layer before physical PWM output.
 *
 * The module accepts position, velocity, and incremental commands and
 * applies the calibrated actuator limits before writing the PWM output.
 *
 * The processing order inside servo_actuator_step() is:
 *
 *   command generation
 *       -> anti-windup
 *       -> velocity limit
 *       -> acceleration limit
 *       -> deadband compensation
 *       -> position clamp
 *       -> PWM output
 *       -> state publication
 *
 * This module is a safety limiter, not a trajectory planner. Higher-level
 * trajectory generation remains responsible for producing the desired
 * motion profile.
 *
 * The module is task-agnostic. A single task should own calls to the
 * servo_actuator_*() APIs at a given time.
 */

#ifndef SERVO_ACTUATOR_H
#define SERVO_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "servo_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Physical actuator limits used by the final velocity and acceleration
 * constraints.
 *
 * SERVO_SLEW_MAX_US_PER_S limits the maximum commanded servo speed.
 * SERVO_ACCEL_MAX_US_PER_S2 limits how quickly the applied speed may change.
 *
 * These values are the single source of truth for the actuator layer and
 * are also used by higher-level trajectory configuration where required.
 */
#define SERVO_SLEW_MAX_US_PER_S    5100.0f
#define SERVO_ACCEL_MAX_US_PER_S2  340000.0f

/**
 * @brief Initialize the actuator layer.
 *
 * Initializes all internal servo states at their calibrated neutral
 * positions and initializes the physical PWM outputs.
 */
void servo_actuator_init(void);

/**
 * @brief Set an absolute servo position target.
 *
 * The target is approached gradually by servo_actuator_step() and is
 * subject to the configured velocity, acceleration, and position limits.
 *
 * Setting a target also selects position mode for the specified channel.
 *
 * @param ch Servo channel.
 * @param target_us Target position in PWM microseconds.
 */
void servo_actuator_set_target(servo_ch_t ch, int32_t target_us);

/**
 * @brief Set a continuous servo velocity command.
 *
 * The servo continues moving at the requested velocity until velocity
 * mode is disabled or another position command is issued.
 *
 * The applied velocity remains subject to the actuator velocity and
 * acceleration limits.
 *
 * @param ch Servo channel.
 * @param us_per_sec Commanded velocity in microseconds per second.
 *                   A negative value commands motion in the opposite direction.
 *                   Zero stops the servo at its current position.
 */
void servo_actuator_set_velocity(servo_ch_t ch, float us_per_sec);

/**
 * @brief Apply an incremental position command.
 *
 * Adds delta_us to the current position target. The updated target is
 * processed by servo_actuator_step(), where all actuator limits are applied.
 *
 * @param ch Servo channel.
 * @param delta_us Position increment in PWM microseconds.
 */
void servo_actuator_apply_delta(servo_ch_t ch, int32_t delta_us);

/**
 * @brief Read the internal actuator positions.
 *
 * This function returns the positions maintained by the actuator layer.
 * It is intended for the task that owns the actuator interface.
 *
 * Other tasks should consume the published actuator state through
 * system_state rather than accessing this internal state directly.
 *
 * @param s1 Output position for servo S1.
 * @param s2 Output position for servo S2.
 * @param s3 Output position for servo S3.
 */
void servo_actuator_get_local(
    int32_t *s1,
    int32_t *s2,
    int32_t *s3
);

/**
 * @brief Apply calibration data to one servo channel.
 *
 * Updates the neutral position, hard position limits, and deadband used
 * by the actuator layer.
 *
 * @param ch Servo channel.
 * @param neutral Calibrated neutral position in PWM microseconds.
 * @param min Minimum allowed position in PWM microseconds.
 * @param max Maximum allowed position in PWM microseconds.
 * @param deadband_us Deadband compensation in PWM microseconds.
 */
void servo_actuator_set_calib(
    servo_ch_t ch,
    int32_t neutral,
    int32_t min,
    int32_t max,
    int32_t deadband_us
);

/**
 * @brief Advance the actuator layer by one control cycle.
 *
 * Converts the active command into a constrained actuator motion, writes
 * the resulting PWM output, and publishes the updated actuator state.
 *
 * This function should be called once per control cycle after all command
 * updates for that cycle have been applied.
 *
 * @param dt Control-loop period in seconds.
 *           Values less than or equal to zero are ignored.
 */
void servo_actuator_step(float dt);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_ACTUATOR_H */

/**
 * @file    servo_pwm.h
 * @brief   Low-level PWM interface for the three servo actuators.
 *
 * This module is the lowest-level servo interface in the application.
 * It is the only layer allowed to access the TIM1 compare registers
 * directly.
 *
 * Higher-level modules such as servo_actuator and trajectory must use
 * servo_pwm_write_us() instead of accessing timer registers directly.
 *
 * Keeping the hardware access isolated here allows the timer, channel,
 * or GPIO mapping to be changed without modifying the higher-level
 * control layers.
 *
 * Hardware mapping:
 *
 *     S1 -> TIM1_CH1 -> PE9
 *     S2 -> TIM1_CH2 -> PE11
 *     S3 -> TIM1_CH3 -> PE13
 *
 * TIM1 configuration:
 *
 *     Prescaler : 199
 *     Period    : 19999
 *     PWM       : 50 Hz
 *     Tick      : 1 us
 *
 * The 1 us timer resolution allows the public API to express servo
 * commands directly in microseconds.
 */

#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Absolute hardware safety limits for standard RC servos.
 *
 * These limits are independent of per-servo mechanical calibration.
 * Mechanical calibration and normal operating limits belong to the
 * higher-level servo_actuator layer.
 *
 * These values therefore act as the final protection boundary before
 * a PWM command reaches the timer hardware.
 */
#define SERVO_US_ABS_MIN             500u
#define SERVO_US_ABS_MAX            2500u
#define SERVO_US_NEUTRAL_DEFAULT    1500u

/*
 * Logical servo channel identifiers.
 *
 * The mapping from these identifiers to the physical timer channels
 * is implemented inside this BSP module.
 */
typedef enum
{
    SERVO_CH_S1 = 0,
    SERVO_CH_S2,
    SERVO_CH_S3,
    SERVO_CH_COUNT
} servo_ch_t;

/**
 * @brief Initialize PWM output for all three servo channels.
 *
 * Starts TIM1 PWM on CH1, CH2, and CH3 and immediately commands all
 * three servos to the neutral position.
 *
 * Initializing the outputs to a known neutral command prevents an
 * unintended actuator movement during power-up before the higher-level
 * control system begins issuing commands.
 *
 * This function should be called once during system initialization.
 */
void servo_pwm_init(void);

/**
 * @brief Write a servo PWM pulse width in microseconds.
 *
 * The requested pulse width is clamped to the absolute hardware
 * safety range before the timer compare register is updated.
 *
 * This clamp is intentionally implemented at the lowest actuator
 * interface so that invalid commands from higher-level software
 * cannot directly produce an out-of-range PWM signal.
 *
 * @param ch  Logical servo channel.
 * @param us  PWM pulse width in microseconds.
 *            1500 us represents the default neutral command.
 */
void servo_pwm_write_us(servo_ch_t ch, uint16_t us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_PWM_H */

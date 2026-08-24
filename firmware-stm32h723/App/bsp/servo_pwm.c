/**
 * @file    servo_pwm.c
 * @brief   Low-level PWM output for servo actuators S1, S2, S3.
 *
 * This module provides the hardware abstraction between the higher-level
 * actuator control logic and the STM32 TIM1 PWM peripheral.
 *
 * The module intentionally keeps all direct timer register access here.
 */

#include "servo_pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

/**
 * @brief Initialize PWM output for all three servo channels.
 *
 * TIM1 CH1, CH2, and CH3 are started independently because all three
 * servos share the same timer but use separate compare channels.
 *
 * After enabling PWM, all channels are immediately set to the default
 * neutral command. This establishes a deterministic actuator state
 * before the servo power stage is enabled or higher-level control
 * commands are applied.
 */
void servo_pwm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* S1 -> PE9  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* S2 -> PE11 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* S3 -> PE13 */

    /*
     * Establish a known neutral command immediately after enabling
     * the PWM outputs.
     *
     * This prevents an unintended pulse width from being applied
     * during startup when the timer compare registers may not yet
     * contain the desired actuator command.
     */
    servo_pwm_write_us(SERVO_CH_S1, SERVO_US_NEUTRAL_DEFAULT);

    servo_pwm_write_us(SERVO_CH_S2, SERVO_US_NEUTRAL_DEFAULT);

    servo_pwm_write_us(SERVO_CH_S3, SERVO_US_NEUTRAL_DEFAULT);
}

/**
 * @brief Write a bounded PWM pulse width to one servo channel.
 *
 * The requested pulse width is first clamped to the absolute hardware
 * safety range. The resulting value is then written directly to the
 * corresponding TIM1 compare register.
 *
 * The function silently ignores invalid channel identifiers. This
 * prevents an invalid logical channel from modifying an unrelated
 * timer channel.
 *
 * @param ch  Logical servo channel.
 * @param us  Requested PWM pulse width in microseconds.
 */
void servo_pwm_write_us(servo_ch_t ch, uint16_t us)
{
    /*
     * Apply the final hardware-level safety clamp.
     *
     * Per-servo calibration limits are handled by the higher-level
     * actuator layer. These limits protect the physical PWM interface
     * regardless of the source of the command.
     */
    if (us < SERVO_US_ABS_MIN)
    {
        us = SERVO_US_ABS_MIN;
    }

    if (us > SERVO_US_ABS_MAX)
    {
        us = SERVO_US_ABS_MAX;
    }

    /*
     * Map the logical servo identifier to its physical TIM1 channel.
     *
     * The timer compare value is expressed directly in microseconds
     * because TIM1 is configured with a 1 MHz counter frequency.
     */
    switch (ch)
    {
        case SERVO_CH_S1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
            break;

        case SERVO_CH_S2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us);
            break;

        case SERVO_CH_S3:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us);
            break;

        default:
            /*
             * Ignore invalid channel identifiers.
             *
             * No timer register is modified when the logical channel
             * does not map to a valid servo output.
             */
            break;
    }
}

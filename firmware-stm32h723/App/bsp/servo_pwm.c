/**
 * @file    servo_pwm.c
 * @brief   Tang 1 - PWM output cho 3 servo S1/S2/S3 (TIM1_CH1/CH2/CH3).
 * @see     servo_pwm.h, PingpongTable_ProfessionalDesign muc B.2.2
 */
#include "servo_pwm.h"
#include "main.h"
//#include "tim.h"   /* CubeMX-generated (Core/Inc/tim.h): extern TIM_HandleTypeDef htim1; */

/* ------------------------------------------------------------------------ */
extern TIM_HandleTypeDef htim1;

void servo_pwm_init(void)
{
    /* Bat PWM ca 3 kenh doc lap tren cung 1 timer advanced */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* S1 - PE9  */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* S2 - PE11 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* S3 - PE13 */

    /* Dua ve neutral ngay lap tuc, truoc khi bat nguon servo, tranh giat
     * luc power-up (CCR mac dinh CubeMX co the la 0 hoac gia tri config cu). */
    servo_pwm_write_us(SERVO_CH_S1, SERVO_US_NEUTRAL_DEFAULT);
    servo_pwm_write_us(SERVO_CH_S2, SERVO_US_NEUTRAL_DEFAULT);
    servo_pwm_write_us(SERVO_CH_S3, SERVO_US_NEUTRAL_DEFAULT);
}

void servo_pwm_write_us(servo_ch_t ch, uint16_t us)
{
    if (us < SERVO_US_ABS_MIN) us = SERVO_US_ABS_MIN;
    if (us > SERVO_US_ABS_MAX) us = SERVO_US_ABS_MAX;

    switch (ch) {
        case SERVO_CH_S1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us); break;
        case SERVO_CH_S2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us); break;
        case SERVO_CH_S3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us); break;
        default: break; /* kenh khong hop le - bo qua, khong ghi gi ca */
    }
}

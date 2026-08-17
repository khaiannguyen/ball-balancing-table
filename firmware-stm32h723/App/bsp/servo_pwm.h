/**
 * @file    servo_pwm.h
 * @brief   Tang 1 - PWM output cho 3 servo S1/S2/S3 (TIM1_CH1/CH2/CH3).
 *
 * Day la lop DUY NHAT duoc phep cham vao thanh ghi timer. Moi lop phia tren
 * (servo_actuator, trajectory) khong bao gio ghi CCRx truc tiep - luon di qua
 * servo_pwm_write_us(). Neu sau nay doi phan cung (khac timer/chan), chi sua
 * file nay, khong anh huong Tang 2/3.
 *
 * Mapping phan cung (xac nhan tu HAL_TIM_MspPostInit, CubeMX):
 *   S1 -> TIM1_CH1 -> PE9
 *   S2 -> TIM1_CH2 -> PE11
 *   S3 -> TIM1_CH3 -> PE13
 *
 * Cau hinh TIM1 (theo Phu luc A.1): PSC=199, ARR=19999 -> 50Hz, tick = 1us
 * (gia dinh timer clock 200MHz -> (199+1) chia -> 1MHz tick).
 *
 * Tham chieu: PingpongTable_ProfessionalDesign, muc 2 va Phu luc B.2.2.
 */
#ifndef SERVO_PWM_H
#define SERVO_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* An toan phan cung TUYET DOI cho moi RC servo pho thong - KHONG lien quan
 * calib co khi tung servo (calib that nam o servo_actuator, muc 12). Day la
 * "lan chan cuoi cung" de khong bao gio xuat xung pha huy servo/co cau. */
#define SERVO_US_ABS_MIN   500u
#define SERVO_US_ABS_MAX  2500u

#define SERVO_US_NEUTRAL_DEFAULT  1500u

typedef enum {
    SERVO_CH_S1 = 0,
    SERVO_CH_S2,
    SERVO_CH_S3,
    SERVO_CH_COUNT
} servo_ch_t;

/**
 * @brief Khoi dong PWM ca 3 kenh TIM1_CH1/CH2/CH3, dua ve neutral 1500us ngay
 *        lap tuc de tranh giat servo luc power-up (truoc khi bat cong suat).
 *        Goi 1 lan duy nhat luc init he thong.
 */
void servo_pwm_init(void);

/**
 * @brief Ghi truc tiep gia tri xung PWM (us) cho 1 kenh servo.
 *
 * Tu dong clamp ve [SERVO_US_ABS_MIN, SERVO_US_ABS_MAX] - day la lop bao ve
 * cuoi cung truoc khi ra phan cung, ke ca khi lop tren tinh sai.
 *
 * @param ch  Kenh servo (SERVO_CH_S1/S2/S3)
 * @param us  Do rong xung, don vi microgiay (vd 1500 = neutral)
 */
void servo_pwm_write_us(servo_ch_t ch, uint16_t us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_PWM_H */

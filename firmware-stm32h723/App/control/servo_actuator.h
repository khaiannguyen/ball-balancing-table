/**
 * @file    servo_actuator.h
 * @brief   Tang 2 - Actuator layer: vi tri tuyet doi, toc do, incremental,
 *          deadband compensation, gioi han van toc + gia toc, anti-windup.
 *
 * Day la module THUAN, khong tu tao FreeRTOS task/timer nao ben trong -
 * task-agnostic. Tai 1 thoi diem chi dung 1 task duoc goi cac ham
 * servo_actuator_*() (nguyen tac single-writer):
 *   - O buoc bring-up: task test tam (servo_test.c).
 *   - O ban chinh thuc: Task_ControlLoop.
 * Chuyen doi giua 2 giai doan tren KHONG sua bat ky dong nao trong file nay.
 *
 * Thu tu bat buoc trong servo_actuator_step(): van toc mong muon -> anti-
 * windup -> gioi han van toc (slew-rate) -> gioi han gia toc (vel_current
 * ramp dan) -> deadband kick -> clamp cung -> ghi PWM -> publish snapshot.
 * Khong duoc dao.
 *
 * VAI TRO trong kien truc 3 tang: day la lop "safety limiter" CUOI CUNG
 * truoc PWM that, KHONG phai trajectory planner - khong tao profile
 * chuyen dong (viec do thuoc ve Trajectory Engine Tang 3, trajectory.h).
 * 2 lop gioi han (Tang 2 o day va Tang 3) KHONG thay the nhau, cung ton
 * tai song song (xem ke hoach sua trajectory/actuator/IK cho servo moi,
 * muc 4).
 */
#ifndef SERVO_ACTUATOR_H
#define SERVO_ACTUATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "servo_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Servo dang dung: 40kg.cm, 0.085s/60do (khong tai, 7.4V), 500-2500us/180do.
 *
 *   us/do              = 2000/180                = 11.111 us/do
 *   V_max ly thuyet     = 60/0.085                = 705.88 do/s
 *   V_max ly thuyet(us) = 705.88 * 11.111         = 7843 us/s
 *
 * SERVO_SLEW_MAX_US_PER_S dung ~65% gia tri tren de con margin khi co tai
 * that (mat ban nghieng, ma sat) va tranh dong dinh gan stall current -
 * BAT BUOC xac nhan lai bang thuc nghiem duoi tai that; neu thay dong dien
 * ap sat stall current lien tuc luc di chuyen binh thuong, giam gia tri
 * nay xuong ngay.
 *
 * SERVO_ACCEL_MAX_US_PER_S2 la UOC LUONG (gia su dat toc do toi da trong
 * ~15ms) - TODO: do dong dien that de hieu chinh lai cho chinh xac.
 *
 * Day la NGUON CHAN LY DUY NHAT cho 2 hang so nay - Trajectory Engine
 * Tang 3 (control_mode_balance.c) tham chieu THANG toi day (vd quy doi
 * TRAJ_HEIGHT_V_MAX/A_MAX qua HEIGHT_K_US_PER_MM), KHONG duoc dinh nghia
 * lai o noi khac de tranh lech gia tri khi tune lai.
 */
#define SERVO_SLEW_MAX_US_PER_S    5100.0f
#define SERVO_ACCEL_MAX_US_PER_S2  340000.0f

/**
 * @brief Khoi tao lop actuator: dua tat ca truc ve gia tri neutral (calib
 *        tam thoi ben duoi), goi servo_pwm_init() ben trong. Goi 1 lan luc
 *        init he thong (sau servo_pwm_init() logic da duoc goi gop o day).
 */
void servo_actuator_init(void);

/**
 * @brief (1) API vi tri tuyet doi. Servo se di chuyen DAN toi target_us,
 *        toc do va gia toc di chuyen bi gioi han (khong nhay tuc thi,
 *        khong giat toc). Dung cho: test tay, ve Home tung buoc nho, hoac
 *        lam "diem den" cho trajectory_update() (Tang 3) moi chu ky.
 */
void servo_actuator_set_target(servo_ch_t ch, int32_t target_us);

/**
 * @brief (2) API toc do. Servo se di chuyen LIEN TUC theo van toc dat truoc
 *        (us/giay, am = lui) cho toi khi goi lai servo_actuator_set_velocity()
 *        voi 0.0f (dung lai dung vi tri hien tai) hoac goi set_target() khac
 *        (chuyen ve che do vi tri). Van bi gioi han gia toc nhu (1).
 */
void servo_actuator_set_velocity(servo_ch_t ch, float us_per_sec);

/**
 * @brief (3) API incremental - cong don mot luong nho vao target hien tai,
 *        CHUA clamp/slew/accel (servo_actuator_step() se xu ly o buoc ke
 *        tiep). Day la ham DUY NHAT ma PID incremental duoc phep goi -
 *        khong bao gio goi thang servo_pwm_write_us().
 */
void servo_actuator_apply_delta(servo_ch_t ch, int32_t delta_us);

/**
 * @brief Doc nhanh gia tri noi bo da ap dung - CHI dung cho debug/test trong
 *        cung task so huu (test task, hoac Task_ControlLoop).
 *        Task KHAC luon phai doc qua actuator_state_get() (system_state.c,
 *        seqlock) - khong bao gio goi ham nay tu task khac.
 */
void servo_actuator_get_local(int32_t *s1, int32_t *s2, int32_t *s3);

/**
 * @brief (4) API vi tri tuyet doi. Buoc xu ly chinh - PHAI goi dung 1 lan,
 *        cuoi moi chu ky dieu khien, sau khi da goi 1 (hoac nhieu) ham
 *        set/apply_delta o tren. Thu tu ben trong: xem doc o dau file.
 *
 * @param dt  Thoi gian tu lan goi truoc, don vi GIAY (vd 0.01f cho 100Hz).
 *            dt <= 0 -> bo qua chu ky nay (khong chia cho 0).
 */
void servo_actuator_step(float dt);

/**
 * @brief Nap lai calib cho 1 truc servo. Gia tri mac dinh tam thoi (xem
 *        servo_actuator.c) nam trong khoang an toan cua servo 40kg.cm
 *        moi. Goi ham nay voi du lieu that doc tu Flash (calibration_data_t)
 *        sau Mode 0 Calibration khi co.
 */
void servo_actuator_set_calib(servo_ch_t ch, int32_t neutral, int32_t min,
                               int32_t max, int32_t deadband_us);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_ACTUATOR_H */

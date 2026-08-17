#ifndef CONTROL_MODE_BALANCE_H
#define CONTROL_MODE_BALANCE_H
#include <stdbool.h>

/**
 * @file    control_mode_balance.h
 * @brief   Mode 2 - Balance.
 *
 * Khong PID/IMU nao chay tren MCU cho mode nay:
 *
 * - Ball:ON  -> nhan Roll_desired/Pitch_desired/Height_desired tu Jetson
 *   qua CAN 0x204 (setpoint_t.Roll_d/Pitch_d/Height_d). Setpoint duoc cho
 *   qua Trajectory Engine (Tang 3, trajectory.h) o KHONG GIAN
 *   Roll/Pitch/Height TRUOC khi goi control_ball_apply_rph() - gioi han
 *   van toc/gia toc thay doi cua chinh Roll/Pitch/Height (khac slew-rate
 *   Tang 2, gioi han o khong gian truc S1/S2/S3 SAU IK). Vi Jetson gui
 *   lien tuc tan so cao, moi khi setpoint moi khac setpoint dang chay do
 *   qua epsilon (TARGET_EPSILON_*, xem control_mode_balance.c),
 *   trajectory_replan() ngay tu trang thai (x,v,a) HIEN TAI - KHONG reset
 *   ve v=0 (diem sua cot loi cua Trajectory Engine, xem trajectory.h) nen
 *   khong giat khi doi setpoint giua luc dang chay.
 * - Ball:OFF / Failsafe (camera_state_is_ok() == false, mat Jetson
 *   >500ms) -> dung LAI CHINH trajectory_replan() voi target=0
 *   (Roll=Pitch=Height=0, ban phang) thay vi reset tuc thi - tranh giat
 *   khi chuyen ON->OFF giua luc dang di chuyen. Dung Vmax/Amax BINH
 *   THUONG (khong phai duong E-stop khan cap) - neu sau nay can dung that
 *   khan cap (nut phan cung), nen di duong rieng khong qua trajectory
 *   engine, goi thang servo_actuator_set_target() ve neutral (chap nhan
 *   giat, doi lay toc do phan ung).
 *
 * Ca 2 nhanh ON va OFF/failsafe dung CHUNG 1 trajectory engine (khong tu
 * viet logic braking rieng o day) - xem control_mode_balance.c.
 *
 * LUU Y: control_mode_balance_step(dt) CAN dt that (dung cho
 * trajectory_update() o CA 2 nhanh) - khong duoc bo qua dt. Dt bat thuong
 * (<=0 hoac qua lon, xem DT_MAX trong control_mode_balance.c) se bi bo qua
 * 1 chu ky thay vi lam trajectory nhay xa dot ngot.
 *
 * Muc dich thiet ke: dung de test/xac nhan anh xa truc IK truoc khi lam
 * PID that ben Jetson (J7) - loai bo hoan toan bien nhieu tu PID noi bo MCU
 * trong luc test.
 */

void control_mode_balance_enter(void);
void control_mode_balance_step(float dt);

/* Toggle Ball ON/OFF - goi tu task_button_ui.c hoac debug console.
 * TODO: neu da co san 1 field/enum khac (vd trong operating_mode_t) dung
 * de phan biet Ball ON/OFF, thay bien static s_ball_on trong
 * control_mode_balance.c bang nguon that do thay vi giu static o day. */
void control_mode_balance_set_ball_on(bool on);
bool control_mode_balance_get_ball_on(void);

#endif /* CONTROL_MODE_BALANCE_H */

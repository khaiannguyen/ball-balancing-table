#ifndef CONTROL_MODE_POSITION_H
#define CONTROL_MODE_POSITION_H

/**
 * @file    control_mode_position.h
 * @brief   Mode 3 - Position (mục 12.3).
 *
 * Kiến trúc GIỐNG HỆT Mode Balance (control_mode_balance.c) về phần MCU:
 * MCU vẫn chỉ là "tay chân" - nhận Roll_desired/Pitch_desired từ Jetson
 * qua CAN 0x204 (setpoint_t.Roll_d/Pitch_d) rồi gọi thẳng
 * control_ball_apply_rp(), KHÔNG tự tính PID gì ở MCU cả.
 *
 * KHÁC Balance ở ĐÚNG 1 điểm: nguồn setpoint bên phía Jetson.
 *   - Balance : Jetson PID luôn nhắm tâm bàn (0,0).
 *   - Position: Jetson PID nhắm (Ballx_d, Bally_d) do NGƯỜI DÙNG chọn qua
 *     UI (screen_gauge_common.c -> PublishBallDesired() -> setpoint_t.
 *     Ballx_d/Bally_d -> gửi qua CAN 0x104 BALL_DESIRED -> Jetson đọc
 *     bằng ball_desired_read() trong task_control_loop.cpp -> PID tính
 *     Roll_d/Pitch_d -> gửi lại CAN 0x204 -> STM32 parse vào setpoint_t.
 *     Roll_d/Pitch_d -> file này áp dụng).
 *
 * Vì vậy MCU KHÔNG cần biết Ballx_d/Bally_d là bao nhiêu để điều khiển -
 * chỉ cần áp Roll_d/Pitch_d giống Balance. Ballx_d/Bally_d chỉ đi NGANG
 * qua MCU (UI ghi -> CAN TX gửi đi mỗi 100Hz, xem task_can_tx.c mục
 * 0x104), không được đọc lại ở control_mode_position.c này.
 *
 * Không có Ball ON/OFF toggle như Balance: người dùng đã chủ động chọn
 * Position + chỉnh Ballx_d/Bally_d qua UI, nghĩa là luôn muốn robot bám
 * theo setpoint đó ngay khi mode đang active (giống tinh thần "BROWSE ->
 * SELECTED = commit" trong screen_gauge_common.c). Failsafe vẫn y hệt
 * Balance: mất camera (camera_state_is_ok()==false) -> tự động về neutral
 * mỗi chu kỳ, không cần lệnh ngoài.
 */

void control_mode_position_enter(void);
void control_mode_position_step(float dt);

#endif /* CONTROL_MODE_POSITION_H */

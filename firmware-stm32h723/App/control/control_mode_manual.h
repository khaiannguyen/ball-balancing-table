#ifndef CONTROL_MODE_MANUAL_H
#define CONTROL_MODE_MANUAL_H
#include <stdint.h>
#include <stdbool.h>

/**
 * @file    control_mode_manual.h
 * @brief   Mode 4 - Manual: test tay servo, dùng để chuẩn bị dữ liệu cho
 *          Mode Calib (deadband) hoặc debug cơ khí độc lập. KHÔNG tự động
 *          chuyển state khi xong (khác Home/Calib) - người dùng tự thoát
 *          bằng nút UI khi thấy đủ, vì đây là công cụ debug không có "đích"
 *          cố định. Xem B6_Control.md mục 0 + 5.4.
 */

typedef enum {
    MANUAL_SUB_IDLE = 0,        // chưa chọn việc gì, chờ người dùng chọn qua UI
    MANUAL_SUB_MANUAL_STEP,     // (1) chỉnh tay us từng servo, log roll/pitch
    MANUAL_SUB_SWEEP_LOG,       // (2) quét toàn dải, log CSV
    MANUAL_SUB_DONE             // sweep vừa xong 1 phiên, chờ chọn việc kế
    /* MANUAL_SUB_DEADBAND_SCAN đã BỎ - cảm biến nhiễu, đo không chính xác */
} manual_sub_state_t;

/** @brief Gọi 1 lần khi Task_ControlLoop phát hiện setpoint.mode vừa đổi
 *         sang OPMODE_MANUAL. Reset về MANUAL_SUB_IDLE, cập nhật guideText. */
void control_mode_manual_enter(void);

/** @brief Gọi mỗi chu kỳ Task_ControlLoop khi setpoint.mode == OPMODE_MANUAL
 *         và system_state == STATE_RUN. */
void control_mode_manual_step(float dt);

manual_sub_state_t control_mode_manual_get_sub_state(void);

/* ---- Gọi từ task_button_ui.c khi đang ở màn UI mode Manual ---- */

/** @brief Đổi sub-state (chọn việc muốn làm). Tự servo_test_stop() phiên cũ
 *         nếu đang chạy trước khi chuyển. */
void control_mode_manual_select_substate(manual_sub_state_t sub);

/** @brief Chỉnh us cho servo đang chọn - chỉ có tác dụng khi sub-state ==
 *         MANUAL_SUB_MANUAL_STEP, các sub-state khác bỏ qua (không lỗi). */
void control_mode_manual_adjust(int16_t delta_us);

/** @brief Đổi kênh đang chỉnh tay (1/2/3) - chỉ có tác dụng khi sub-state ==
 *         MANUAL_SUB_MANUAL_STEP. */
void control_mode_manual_select_channel(uint8_t servo_ch);

#endif /* CONTROL_MODE_MANUAL_H */

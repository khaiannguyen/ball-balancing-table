#ifndef CONTROL_MODE_HOME_H
#define CONTROL_MODE_HOME_H
#include <stdbool.h>

/**
 * @file    control_mode_home.h
 * @brief   Mode 0 - Home: đưa 3 servo về vị trí neutral (S1/S2/S3_neutral,
 *          đọc từ calibration_data), xong tự gửi EVT_HOME_DONE để
 *          Task_StateMachine chuyển RUN -> READY (mục 12 THIẾT KẾ MODE
 *          VẬN HÀNH, B6_Control.md mục 5.2).
 *
 * SỬA (Giai đoạn 5 - áp dụng trajectory_start_synced3): thay vì set_target()
 * 1 lần rồi phó mặc hoàn toàn cho slew-rate limiter trong servo_actuator_step()
 * tự lo việc đi êm, giờ chủ động sinh 1 profile hình thang ĐỒNG BỘ cho cả 3
 * servo (cùng bắt đầu, cùng kết thúc lúc t_total = max(T0_1,T0_2,T0_3)) - xem
 * trajectory.h. LƯU Ý: tốc độ v_max của trajectory PHẢI <= tốc độ tối đa mà
 * slew-rate limiter trong servo_actuator_step() cho phép, nếu không tín hiệu
 * sẽ bị nắn lại 2 lần chồng nhau (trajectory + slew limiter), phá mất tính
 * "3 trục tới cùng lúc".
 *
 * KHÔNG bao giờ tự gọi system_state_publish() - chỉ gửi event qua
 * StateRequestQueueHandle, đúng nguyên tắc "chỉ Task_StateMachine ghi
 * system_state" (xem task_state_machine.c).
 */

/**
 * @brief Gọi 1 lần khi Task_ControlLoop phát hiện setpoint.mode vừa đổi
 *        sang OPMODE_HOME (mode_changed == true). Đọc vị trí THẬT hiện tại
 *        (servo_actuator_get_local()) làm điểm xuất phát, khởi tạo
 *        trajectory_start_synced3() để cả 3 servo cùng bắt đầu và cùng
 *        tới neutral 1 lúc, dù quãng đường mỗi trục khác nhau.
 */
void control_mode_home_enter(void);

/**
 * @brief Gọi mỗi chu kỳ Task_ControlLoop khi setpoint.mode == OPMODE_HOME
 *        và system_state == STATE_RUN. Chạy trajectory_update() cho tới khi
 *        cả 3 trục xong (trajectory_is_done()), sau đó tự phát hiện "đã tới
 *        home" (debounce HOME_SETTLE_CYCLES chu kỳ liên tiếp trong dung sai
 *        HOME_TOLERANCE_US, dựa trên vị trí THẬT đọc được - không phải dựa
 *        vào trajectory báo done) rồi gửi EVT_HOME_DONE. Tự gọi
 *        control_mode_home_enter() nếu chưa được gọi (an toàn nếu
 *        Task_ControlLoop quên gọi enter() riêng).
 */
void control_mode_home_step(float dt);

/**
 * @brief true sau khi đã gửi EVT_HOME_DONE thành công trong lần chạy hiện
 *        tại của mode này. Dùng cho debug/log, không bắt buộc dùng.
 */
bool control_mode_home_is_done(void);

#endif /* CONTROL_MODE_HOME_H */

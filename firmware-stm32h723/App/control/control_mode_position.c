#include "control_mode_position.h"
#include "control_ball_common.h"
#include "system_state.h"
#include <stdio.h>

/* Xem giải thích kiến trúc đầy đủ trong control_mode_position.h. Tóm tắt:
 * MCU chỉ áp Roll_d/Pitch_d nhận từ Jetson (setpoint_t, ghi bởi
 * task_can_rx.c khi parse CAN 0x204) - Jetson đã tự PID nhắm tới
 * (Ballx_d, Bally_d) do UI chọn. File này KHÔNG đọc Ballx_d/Bally_d,
 * không có PID/IMU nào ở MCU, giống hệt nhánh Ball:ON của
 * control_mode_balance.c. */

void control_mode_position_enter(void)
{
    /* Không có cờ Ball ON/OFF như Balance - vào mode là áp dụng ngay
     * Roll_d/Pitch_d hiện có trong setpoint_t. Không reset gì ở đây vì
     * Ballx_d/Bally_d (và do đó Roll_d/Pitch_d suy ra từ nó) là trạng thái
     * người dùng đang chỉnh dở từ UI (screen_gauge_common.c) - KHÔNG được
     * xoá khi chuyển mode qua lại, khác với lý do Balance phải ép về
     * Ball:OFF (Balance dùng cờ on/off nội bộ dễ "giật" nếu giữ nguyên
     * lệnh cũ của lần RUN trước; Position không có cờ đó nên không có
     * rủi ro tương tự). Failsafe camera_state_is_ok() bên dưới đã đủ an
     * toàn cho trường hợp mất kết nối Jetson ngay sau khi vào mode. */
}

void control_mode_position_step(float dt)
{
    (void)dt;   /* không PID ở MCU, giữ tham số để khớp chữ ký chung với
                   các control_mode_*.c khác (state machine gọi đồng nhất) */

    bool camera_ok = camera_state_is_ok();

    if (camera_ok) {
        /* ---- Nhận thẳng Roll_d/Pitch_d từ Jetson, áp ngay ---- */
        setpoint_t sp;
        if (setpoint_get(&sp)) {
            control_ball_apply_rp(sp.Roll_d, sp.Pitch_d);
        }
        /* Miss mutex (timeout 5 tick) -> KHÔNG áp gì cả, giữ nguyên lệnh
         * servo chu kỳ trước - cùng nguyên tắc "không áp nửa vời" như
         * task_can_rx.c (0x204) và control_mode_balance.c. */

    } else {
        /* ---- Mất kết nối Jetson (failsafe) ----
         * Về neutral THẲNG, giống hệt nhánh Ball:OFF/failsafe của
         * control_mode_balance.c - không dùng lại Roll_d/Pitch_d cũ có
         * thể đã stale. */
        control_ball_apply_rp(0.0f, 0.0f);
    }
}

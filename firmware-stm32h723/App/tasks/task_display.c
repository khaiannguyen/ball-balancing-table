#include "cmsis_os2.h"
#include <stdbool.h>

#include "screen.h"
#include "screen_boot.h"
#include "screen_home.h"
#include "screen_calibrate.h"
#include "screen_balance.h"
#include "screen_position.h"
#include "screen_manual.h"
#include "screen_fault.h"
#include "tft_service.h"
#include "ui_data.h"
#include "system_state.h"

extern osEventFlagsId_t SystemEventGroupHandle;   /* THÊM (B7): chờ EVT_BIT_BOOT_DONE */

/* =========================================================
 * Task_Display - priority 1 (THẤP NHẤT, mục 3.1/2).
 *
 * KHÔNG gọi task_alive_mark() ở đây - ALIVE_MASK_EXPECTED (system_state.h)
 * chỉ gồm đúng 3 bit (CONTROL_LOOP|IMU_FUSION|CAN_RX). Thêm bit thứ 4 sẽ
 * làm watchdog đòi 1 bit không có trong mask mong đợi -> không bao giờ
 * refresh IWDG -> MCU tự reset liên tục. Task_CAN_TX cũng cố tình KHÔNG
 * gọi hàm này vì cùng lý do (xem comment trong task_can_tx.c).
 * ========================================================= */

#define DISPLAY_PERIOD_MS   (1000 / 25)   /* 25Hz, theo mục 3.1 */

/* THÊM (B7): thời gian tối thiểu màn Boot phải đứng yên để người dùng đọc
 * kịp log, kể cả khi init (Task_ControlLoop) chạy xong trong vài chục ms.
 * Không dùng osDelay() cứng TRƯỚC khi chờ event (như bản cũ) vì vậy sẽ lại
 * biến thành "chờ mù" bất kể EVT_BIT_BOOT_DONE - thay vào đó: chờ event
 * trước (không giới hạn), rồi nếu thời gian trôi qua ít hơn mức tối thiểu,
 * ngủ bù phần còn thiếu. Cách này vẫn phản ứng ngay khi có FAULT/init xong,
 * chỉ đảm bảo người dùng luôn thấy màn Boot ít nhất chừng đó lâu. */
#define BOOT_MIN_DISPLAY_MS  1500u

void StartTaskDisplay(void *argument)
{
    (void)argument;

    TFT_Init();
    UiData_Init();

    /* Đăng ký label nút cho 5 gauge screen */
    ScreenHome_Get();
    ScreenCalibrate_Get();
    ScreenBalance_Get();
    ScreenPosition_Get();
    ScreenManual_Get();

    ScreenManager_Goto(ScreenBoot_Get());

    /* THÊM (B7, fix deadlock): báo cho Task_ControlLoop biết TFT đã init +
     * khung Boot đã vẽ xong - Task_ControlLoop PHẢI chờ bit này trước khi
     * gọi ScreenBoot_AddLog() lần đầu (xem giải thích đầy đủ trong
     * system_state.h, EVT_BIT_TFT_READY). Đặt NGAY SAU ScreenManager_Goto()
     * ở trên - đây là điểm sớm nhất TFT_DrawText() từ nơi khác gọi vào mới
     * an toàn. */
    osEventFlagsSet(SystemEventGroupHandle, EVT_BIT_TFT_READY);

    uint32_t bootScreenStartTick = osKernelGetTickCount();

    /* SỬA (B7): chờ Task_ControlLoop báo init xong thật (IMU + calib +
     * servo_actuator_init() + self-test, xem EVT_BIT_BOOT_DONE trong
     * task_control_loop.c), thay vì osDelay(2000) cứng - không còn phụ
     * thuộc 1 con số đoán mò cho phần "chờ init xong". osFlagsWaitAny +
     * clear-on-read mặc định (KHÔNG dùng osFlagsNoClear như
     * system_state_fault_is_set() - ở đây là sự kiện 1 lần lúc boot, không
     * phải trạng thái cần đọc lặp lại). */
    osEventFlagsWait(SystemEventGroupHandle, EVT_BIT_BOOT_DONE,
                      osFlagsWaitAny, osWaitForever);

    /* THÊM (B7): nếu init xong quá nhanh (thực tế chỉ vài chục ms), ngủ bù
     * phần còn thiếu để màn Boot đứng đủ BOOT_MIN_DISPLAY_MS - tránh hiện
     * tượng "sượt qua", người dùng không kịp đọc log nào. Dùng phép trừ
     * unsigned nên vẫn đúng nếu tick counter wrap-around. */
    uint32_t elapsedMs = (osKernelGetTickCount() - bootScreenStartTick)
                         * 1000u / osKernelGetTickFreq();
    if (elapsedMs < BOOT_MIN_DISPLAY_MS) {
        osDelay(BOOT_MIN_DISPLAY_MS - elapsedMs);
    }

    ScreenManager_Goto(ScreenHome_Get());

    bool wasFault = false;
    uint32_t lastWake = osKernelGetTickCount();

    for (;;)
    {
        /* Đổ dữ liệu THẬT từ system_state.h vào g_uiData TRƯỚC khi vẽ -
         * đây là chỗ khiến Roll/Pitch/S1-3/Ball trên màn hình là số
         * thật, không phải 0 tĩnh như trước khi có UiData_SyncFromSystemState(). */
        UiData_SyncFromSystemState();

        bool fault = system_state_fault_is_set();

        if (fault && !wasFault)
        {
            /* rising edge: đẩy screen hiện tại vào stack, hiện Fault.
             * Nhờ ScreenManager kiểu stack, dù đang đứng ở Stop/Shutdown
             * cũng được nhớ đúng, GoBack() lúc hết lỗi trả về đúng chỗ. */
            ScreenManager_GotoAndRemember(ScreenFault_Get());
        }
        else if (!fault && wasFault)
        {
            ScreenManager_GoBack();
        }
        wasFault = fault;

        ScreenManager_Update();

        lastWake += DISPLAY_PERIOD_MS;
        osDelayUntil(lastWake);
    }
}

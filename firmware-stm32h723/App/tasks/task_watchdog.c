#include "task_watchdog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"                  /* hiwdg1, sinh bởi CubeMX (mục A.1) */

#include "system_state.h"

extern IWDG_HandleTypeDef hiwdg1;

void StartTaskWatchdog(void *argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100);   /* 10Hz, đúng mục 3.1 */

    for (;;)
    {
        vTaskDelayUntil(&lastWake, period);

        /* Đọc mask hiện tại VÀ clear về 0 trong 1 thao tác atomic (đã có sẵn trong
         * system_state.c) - tránh race giữa lúc đọc và lúc clear (mục 3.6). */
        uint32_t mask = task_alive_snapshot_and_clear();

        if (mask == ALIVE_MASK_EXPECTED) {
            HAL_IWDG_Refresh(&hiwdg1);
        }
        /* else: KHÔNG refresh. Đây là hành vi ĐÚNG THIẾT KẾ (mục 3.6, đã ghi rõ ở
         * Phụ lục A.2): nếu 1 trong 3 task quan trọng (ControlLoop/IMU_Fusion/CAN_RX)
         * bị treo/deadlock và không set bit của nó trong 100ms, mask sẽ thiếu bit ->
         * không kick -> IWDG hết giờ -> MCU tự reset toàn bộ, thay vì để servo giữ
         * nguyên vị trí cũ vô thời hạn trong lúc hệ thống thực ra đã "chết".
         *
         * KHÔNG thêm log/print ở nhánh else để "biết lý do" trước khi reset trừ khi
         * bạn chắc chắn thao tác log không tự nó block/treo - nếu không sẽ tự phá
         * mất chức năng bảo vệ mà Task_Watchdog đang cung cấp. */
    }
}

/* =====================================================================
 * TASK TẠM THỜI CHO BRING-UP (mục 8, bước 4) — KHÔNG phải task chính thức.
 * Mục đích: giả lập Jetson gửi frame 0x200-0x204,0x2FF để test đường
 * Task_CAN_RX (parse) + failsafe heartbeat (mục 4.4) qua Logic Analyzer,
 * TRƯỚC KHI có ESP32/Jetson thật.
 *
 * XOÁ FILE NÀY (và lời gọi tương ứng trong StartDisplay/StartControlLoop)
 * khi chuyển sang test với ESP32 thật.
 * ===================================================================== */

#include "task_can_bringup_test.h"   /* thêm dòng này - khớp prototype với .h */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "can_protocol.h"
#include "system_state.h"            /* thêm dòng này - cần cho task_alive_mark/ALIVE_BIT_* */

extern FDCAN_HandleTypeDef hfdcan1;

static bool send_fake(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier        = id;
    h.IdType             = FDCAN_STANDARD_ID;
    h.TxFrameType         = FDCAN_DATA_FRAME;
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch       = FDCAN_BRS_OFF;
    h.FDFormat            = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    switch (dlc) {
        case 1: h.DataLength = FDCAN_DLC_BYTES_1; break;
        case 4: h.DataLength = FDCAN_DLC_BYTES_4; break;
        case 6: h.DataLength = FDCAN_DLC_BYTES_6; break;
        default: h.DataLength = FDCAN_DLC_BYTES_8; break;
    }
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, (uint8_t *)data) == HAL_OK;
}

void StartTaskCanBringupTest(void *argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t   cycle = 0;
    bool       stop_heartbeat = false;   /* đổi thành true bằng debugger (watch) để test failsafe */

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
        cycle++;

        /* ---- Giả 0x204 ATTITUDE_DESIRED — giá trị CỐ ĐỊNH, biết trước để đối
         * chiếu tay với hex bắt được trên Logic Analyzer:
         *   Roll_d  = 12.34 deg  -> x100 = 1234        -> 0x04D2 LE = D2 04
         *   Pitch_d = -5.00 deg  -> x100 = -500         -> 0xFE0C LE = 0C FE
         *   Height_d= 150 mm     -> không scale          -> 0x0096 LE = 96 00
         * => Cả frame 6 byte trên LA phải đọc đúng: D2 04 0C FE 96 00 */
        if (cycle % 5 == 0) {   /* mỗi 500ms */
            uint8_t buf[6];
            can_wr_i16le(&buf[0], 1234);
            can_wr_i16le(&buf[2], -500);
            can_wr_i16le(&buf[4], 150);
            send_fake(CAN_ID_ATTITUDE_DESIRED, 6, buf);
        }

        /* ---- Giả 0x200 BALL_POS: Ballx=100mm, Bally=-50mm ---- */
        if (cycle % 7 == 0) {
            uint8_t buf[4];
            can_wr_i16le(&buf[0], 100);
            can_wr_i16le(&buf[2], -50);
            send_fake(CAN_ID_BALL_POS, 4, buf);
        }

        /* ---- Giả 0x202 BALL_STATE: detected = 1 ---- */
        if (cycle % 7 == 0) {
            uint8_t buf[1] = {1};
            send_fake(CAN_ID_BALL_STATE, 1, buf);
        }

        /* ---- Giả 0x2FF HEARTBEAT — TẮT dòng này (đặt stop_heartbeat=true qua
         * debugger) để xem bit FAULT trong frame 0x103 ROBOT_STATE (do
         * Task_CAN_TX gửi) chuyển 0 -> 1 đúng ~200ms sau, quan sát trực tiếp
         * trên Logic Analyzer, không cần code gì thêm (mục 4.4). ---- */
        if (!stop_heartbeat) {
            uint8_t buf[1] = {(uint8_t)cycle};
            send_fake(CAN_ID_HEARTBEAT_RX, 1, buf);
        }

        /* ==== CHỈ CẦN DÒNG NÀY NẾU BẠN GỌI HÀM NÀY TỪ StartControlLoop ====
         * Task_ControlLoop nằm trong ALIVE_MASK_EXPECTED (mục 3.6) - thiếu
         * dòng này, Task_Watchdog sẽ không bao giờ kick IWDG -> MCU tự reset
         * liên tục (đúng lỗi bạn đã gặp trước đây).
         * NẾU bạn gọi từ StartDisplay thay vì StartControlLoop: XOÁ dòng dưới,
         * vì DisplayTask không có ALIVE_BIT tương ứng, gọi nhầm bit sẽ không
         * gây lỗi biên dịch nhưng sai ý nghĩa (đang "giả vờ" ControlLoop sống
         * trong khi thực ra ControlLoop không chạy gì cả). */
        task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
    }
}

#include "task_can_tx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"                 /* hfdcan1, sinh bởi CubeMX (mục A.1: Classic Frame, IT0) */

#include "system_state.h"
#include "can_protocol.h"
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern osEventFlagsId_t    SystemEventGroupHandle;



/* Đổi số byte (0..8) -> macro FDCAN_DLC_BYTES_x của HAL, vì bảng mục 4.2 quy định
 * DLC cụ thể cho từng ID (không phải lúc nào cũng 8) -> tiết kiệm bus load thật sự. */
static uint32_t dlc_to_fdcan(uint8_t n)
{
    switch (n) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

/* Peek 1 bit trong Event Group mà KHÔNG clear nó (osFlagsNoClear) — dùng để đọc
 * FAULT cho ROBOT_STATE (0x103) mà không ảnh hưởng Task_ControlLoop cũng đang đọc bit này. */
static bool event_flag_is_set(osEventFlagsId_t h, uint32_t bit)
{
    uint32_t r = osEventFlagsWait(h, bit, osFlagsWaitAny | osFlagsNoClear, 0);
    if ((r & osFlagsError) != 0) return false;   /* timeout 0 -> bit chưa set */
    return (r & bit) != 0;
}

static bool can_send(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef h = {0};
    h.Identifier          = id;
    h.IdType               = FDCAN_STANDARD_ID;
    h.TxFrameType           = FDCAN_DATA_FRAME;
    h.DataLength            = dlc_to_fdcan(dlc);
    h.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch         = FDCAN_BRS_OFF;      /* Classic CAN, mục 4.1 - test với ESP32 */
    h.FDFormat              = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    h.MessageMarker         = 0;

    /* Non-blocking: nếu Tx FIFO đầy (bus lỗi/nghẽn), bỏ frame này, KHÔNG chờ —
     * Task_CAN_TX priority 3, không được phép bị block vì lý do phần cứng ngoài. */
    //return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, (uint8_t *)data) == HAL_OK;
    bool ok = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &h, (uint8_t *)data) == HAL_OK;
        //if (!ok) printf("CAN TX FAIL id=0x%lX\r\n", id);   // thêm dòng này
        return ok;
}

void StartTaskCanTx(void *argument)
{
    (void)argument;
    printf("StartTaskCanTx STARTED\r\n");

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);   /* 100Hz, mục 3.1 (chọn 100 thay vì 200 để dư margin bus) */
    uint8_t heartbeat_counter = 0;

    for (;;)
    {
/*
    	FDCAN_ProtocolStatusTypeDef ps;
    	HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    	printf("FDCAN state=%d busoff=%ld ep=%ld ew=%ld TEC=%ld REC=%ld free=%lu\r\n",
    	       hfdcan1.State, ps.BusOff, ps.ErrorPassive, ps.Warning,
    	       ps.LastErrorCode, ps.DataLastErrorCode,
    	       HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1));
*/
        vTaskDelayUntil(&lastWake, period);

        uint8_t buf[8];

        /* ---- 0x100 ATTITUDE: roll, pitch, height (int16, x100 / mm) ---- */
        {
            float roll, pitch, vroll, vpitch;
            imu_state_read(system_state_get_imu_ptr(), &roll, &pitch, &vroll, &vpitch);

            /* Height hiện tại thuộc setpoint (Height_d) - chưa có "actual height" đo
             * riêng trong system_state.h (bàn không có cảm biến độ cao trực tiếp).
             * Tạm dùng Height_d làm giá trị report; sửa lại nếu bạn có nguồn khác. */
            setpoint_t sp;
            float height = 0.0f;
            if (setpoint_get(&sp)) height = sp.Height_d;

            can_wr_i16le(&buf[0], (int16_t)(roll  * 100.0f));
            can_wr_i16le(&buf[2], (int16_t)(pitch * 100.0f));
            can_wr_i16le(&buf[4], (int16_t)height);
            can_send(CAN_ID_ATTITUDE, 6, buf);

            /* ---- 0x101 RATE: vroll, vpitch (int16, x100 deg/s) ---- */
            can_wr_i16le(&buf[0], (int16_t)(vroll  * 100.0f));
            can_wr_i16le(&buf[2], (int16_t)(vpitch * 100.0f));
            can_send(CAN_ID_RATE, 4, buf);
        }

        /* ---- 0x102 SERVO_POS: S1,S2,S3 (uint16, 1us) ---- */
        {
            int32_t s1, s2, s3;
            actuator_state_read(system_state_get_actuator_ptr(), &s1, &s2, &s3);
            can_wr_u16le(&buf[0], (uint16_t)s1);
            can_wr_u16le(&buf[2], (uint16_t)s2);
            can_wr_u16le(&buf[4], (uint16_t)s3);
            can_send(CAN_ID_SERVO_POS, 6, buf);
        }

        /* ---- 0x103 ROBOT_STATE: mode(u8), run_stop|fault bitfield(u8) ---- */
        {
            setpoint_t sp;
            uint8_t mode = 0;
            if (setpoint_get(&sp)) mode = sp.mode;

            uint8_t bits = 0;
            if (system_state_get() == STATE_RUN) bits |= ROBOT_STATE_BIT_RUN;
            if (event_flag_is_set(SystemEventGroupHandle, EVT_BIT_FAULT)) bits |= ROBOT_STATE_BIT_FAULT;
            /* THÊM: báo lại cho bus (debug/giám sát ngoài) biết STM32 có đang
             * nhận heartbeat camera (0x200/0x201) hay không - nguồn thật lấy
             * từ camera_state_is_ok() (system_state.c), cùng cơ chế cameraOk
             * hiển thị trên TFT (ui_data.c). ROBOT_STATE_BIT_CAM_OK cần thêm
             * vào can_protocol.h (xem hướng dẫn kèm theo). */
            if (camera_state_is_ok()) bits |= ROBOT_STATE_BIT_CAM_OK;

            buf[0] = mode;
            buf[1] = bits;
            can_send(CAN_ID_ROBOT_STATE, 2, buf);
        }

        /* ---- 0x104 BALL_DESIRED: Ballx_d, Bally_d, Ballheight_d (int16, mm) ---- */
        {
            setpoint_t sp;
            if (setpoint_get(&sp)) {
                can_wr_i16le(&buf[0], (int16_t)sp.Ballx_d);
                can_wr_i16le(&buf[2], (int16_t)sp.Bally_d);
                /* setpoint_t (system_state.h) hiện CHƯA có Ballheight_d - gửi tạm 0.
                 * Nếu Mode 2 (mục 12.3) cần điều khiển độ cao bóng, thêm trường này
                 * vào setpoint_t rồi thay dòng dưới. */
                can_wr_i16le(&buf[4], 0);
                can_send(CAN_ID_BALL_DESIRED, 6, buf);
            }
        }

        /* ---- 0x1FF HEARTBEAT: counter u8, wrap 0-255 ---- */
        buf[0] = heartbeat_counter++;
        can_send(CAN_ID_HEARTBEAT_TX, 1, buf);

        /* Lưu ý: Task_CAN_TX KHÔNG nằm trong ALIVE_MASK_EXPECTED (mục 3.6 chỉ định nghĩa
         * bit cho ControlLoop/IMU_Fusion/CAN_RX) -> không cần/không được gọi task_alive_mark()
         * ở đây, nếu không watchdog sẽ đòi bit không tồn tại trong mask mong đợi. */
    }
}

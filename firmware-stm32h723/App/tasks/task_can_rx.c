#include "task_can_rx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "main.h"          /* thêm dòng này — cần cho FDCAN_HandleTypeDef, hfdcan1 */
#include <stdio.h>
#include "control_mode_balance.h"


#include "system_state.h"
#include "can_protocol.h"

extern QueueHandle_t      CanRxQueueHandle;
extern osEventFlagsId_t   SystemEventGroupHandle;
extern FDCAN_HandleTypeDef hfdcan1;   /* thêm dòng này */
extern osMessageQueueId_t StateRequestQueueHandle;   /* THÊM (mục 3): cùng handle mà
                                                         task_state_machine.c/task_button_ui.c
                                                         đang dùng - gửi state_event_t vào đây */

/* ==== Phần ISR — dán từ can_rx_isr.c vào đây ==== */
static uint8_t fdcan_dlc_to_bytes(uint32_t dlc_field)
{
    switch (dlc_field) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        default:                return 8;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) return;
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef rxHeader;
    can_frame_t frame;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data) != HAL_OK) {
        return;
    }
    frame.id  = rxHeader.Identifier;
    frame.dlc = fdcan_dlc_to_bytes(rxHeader.DataLength);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(CanRxQueueHandle, &frame, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* ==== Hết phần ISR ==== */
void StartTaskCanRx(void *argument)
{
    (void)argument;
    //printf("StartTaskCanRx STARTED\r\n");

    can_frame_t   frame;
    TickType_t    last_heartbeat_tick = xTaskGetTickCount();
    bool          fault_latched = false;
    setpoint_t    sp;

    for (;;)
    {
        /* Timeout 50ms (không block vô hạn) để vòng lặp vẫn chạy đều đặn kể cả
         * khi không có frame mới -> vừa kiểm tra failsafe heartbeat (mục 4.4)
         * vừa đánh dấu "còn sống" cho Task_Watchdog (mục 3.6) đúng chu kỳ. */
    	FDCAN_ProtocolStatusTypeDef ps;
    	HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    	static uint32_t s_busoff_count = 0;
    	if (ps.BusOff) {
    	    s_busoff_count++;
    	    //printf("FDCAN BUS-OFF detected (lan thu %lu) - tu phuc hoi...\r\n", s_busoff_count);
    	    HAL_FDCAN_Stop(&hfdcan1);
    	    HAL_FDCAN_Start(&hfdcan1);
    	}
        BaseType_t got = xQueueReceive(CanRxQueueHandle, &frame, pdMS_TO_TICKS(50));

        if (got == pdTRUE)
        {
            switch (frame.id)
            {
            case CAN_ID_BALL_POS:                                   /* 0x200, mục 4.3 */
                if (frame.dlc >= 4) {
                    int16_t x = can_rd_i16le(&frame.data[0]);
                    int16_t y = can_rd_i16le(&frame.data[2]);
                    //printf("RX 0x200: Ballx=%d Bally=%d\r\n", x, y);
                    ball_state_write_pos(system_state_get_ball_ptr(), x, y);
                    camera_heartbeat_mark();   /* THÊM: 0x200 chứng tỏ Jetson/camera còn sống */
                }
                break;

            case CAN_ID_BALL_VEL:                                   /* 0x201 */
                if (frame.dlc >= 4) {
                    int16_t vx = can_rd_i16le(&frame.data[0]);
                    int16_t vy = can_rd_i16le(&frame.data[2]);
                    ball_state_write_vel(system_state_get_ball_ptr(), vx, vy);
                    camera_heartbeat_mark();   /* THÊM: 0x201 cũng tính là còn sống */
                }
                break;

            case CAN_ID_BALL_STATE:                                 /* 0x202 */
                if (frame.dlc >= 1) {
                	//printf("RX 0x202: detected=%d\r\n", frame.data[0]);
                    ball_state_write_detected(system_state_get_ball_ptr(), frame.data[0]);

                    control_mode_balance_set_ball_on(frame.data[0] != 0);
                }
                break;

            case CAN_ID_SERVO_CALIB:                                /* 0x203 */
                /* Mục 12.1: nguồn calib CHÍNH THỨC là thuật toán closed-loop nội bộ
                 * (dùng IMU), chạy trong state CALIBRATION. Frame này chỉ là kênh
                 * dự phòng cho override từ tool bên Jetson — ở bước B.3 CHƯA áp dụng
                 * trực tiếp (tránh ghi đè calib bằng dữ liệu sai/giả), chỉ decode để
                 * sẵn sàng dùng khi bạn thiết kế cơ chế xác nhận (vd chỉ nhận khi
                 * state == CALIBRATION, mục 14.1). */
            	printf("RX 0x203: dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            	           frame.dlc,
            	           frame.data[0], frame.data[1], frame.data[2], frame.data[3],
            	           frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                if (frame.dlc >= 6) {
                    uint16_t s1c = can_rd_u16le(&frame.data[0]);
                    uint16_t s2c = can_rd_u16le(&frame.data[2]);
                    uint16_t s3c = can_rd_u16le(&frame.data[4]);
                    (void)s1c; (void)s2c; (void)s3c;
                    /* TODO: áp dụng có điều kiện khi có cơ chế calib override thật */
                }
                break;

            case CAN_ID_ATTITUDE_DESIRED:                           /* 0x204 */
                if (frame.dlc >= 6) {
                    float roll_d   = can_rd_i16le(&frame.data[0]) / 100.0f;
                    float pitch_d  = can_rd_i16le(&frame.data[2]) / 100.0f;
                    float height_d = (float)can_rd_i16le(&frame.data[4]);
                    //printf("CAN_ID_ATTITUDE_DESIRED:\r\n");
                    //printf("RX 0x204: roll=%.2f pitch=%.2f height=%.0f\r\n", roll_d, pitch_d, height_d);

                    /* Read-modify-write: chỉ đổi 3 trường liên quan, giữ nguyên
                     * mode/Ballx_d/Bally_d đang có (setpoint_get/set copy NGUYÊN struct,
                     * mục 3.2). Timeout 5 tick của mutex -> nếu miss, bỏ qua lần này,
                     * KHÔNG áp dụng nửa vời. */
                    if (setpoint_get(&sp)) {
                        sp.Roll_d   = roll_d;
                        sp.Pitch_d  = pitch_d;
                        sp.Height_d = height_d;
                        setpoint_set(&sp);
                    }
                }
                break;

            case CAN_ID_HEARTBEAT_RX:                               /* 0x2FF */
                last_heartbeat_tick = xTaskGetTickCount();
                if (fault_latched) {
                    /* Jetson đã "sống lại" -> tự phục hồi khỏi FAULT do mất heartbeat.
                     * Nếu bạn muốn FAULT phải xác nhận thủ công (đúng tinh thần
                     * SAFE_MODE không tự thoát, mục 14.1), xoá đoạn clear bit này. */
                    osEventFlagsClear(SystemEventGroupHandle, EVT_BIT_FAULT);
                    fault_latched = false;
                    //printf("FAULT CLEARED - heartbeat tro lai!\r\n");
                }
                break;

            default:
                break;  /* ID không thuộc bảng mục 4.3 -> bỏ qua, không phải lỗi */
            }
        }

        /* ---- Failsafe / Timeout, mục 4.4 ----
         * Chạy MỖI vòng lặp (kể cả khi không có frame mới) vì timeout của
         * xQueueReceive (50ms) đảm bảo vòng lặp không "đứng hình" chờ vô hạn. */
        if ((xTaskGetTickCount() - last_heartbeat_tick) > pdMS_TO_TICKS(CAN_HEARTBEAT_TIMEOUT_MS)) {
            if (!fault_latched) {
                osEventFlagsSet(SystemEventGroupHandle, EVT_BIT_FAULT);
                fault_latched = true;
                printf("FAULT SET - mat heartbeat!\r\n");
                /* Task_ControlLoop tự kiểm tra bit FAULT mỗi chu kỳ và chuyển sang
                 * chế độ an toàn (Roll=Pitch=0, Height mặc định) - KHÔNG dùng lại
                 * setpoint cũ có thể đã stale, đúng thiết kế mục 4.4. */

                /* THÊM (mục 3): nối EVT_BIT_FAULT (osEventFlags, UI/CAN_TX đọc) với
                 * EVT_FAULT_DETECTED (state_event_t qua queue, Task_StateMachine đọc)
                 * - 2 cơ chế này trước đây tách rời nhau, Task_StateMachine chưa bao
                 * giờ thực sự vào STATE_ERROR/SAFE_MODE khi mất CAN. Chỉ gửi ở rising
                 * edge (if (!fault_latched) đã true ở trên) - không spam queue mỗi
                 * 50ms trong khi vẫn đang mất heartbeat. Timeout 0: không được phép
                 * block Task_CAN_RX nếu lỡ queue đầy - bỏ qua lần này còn hơn treo task. */
                state_event_t fault_evt = EVT_FAULT_DETECTED;
                osMessageQueuePut(StateRequestQueueHandle, &fault_evt, 0, 0);
            }
        }

        task_alive_mark(ALIVE_BIT_CAN_RX);   /* báo sống cho Task_Watchdog, mục 3.6 */
        //printf("StartTaskCanRx ENDED\r\n");
    }
}

/**
 * @file    task_can_rx.c
 * @brief   CAN receive task and heartbeat supervision.
 *
 * Receives CAN frames from the FDCAN interrupt path, decodes application
 * messages, updates shared system state, and supervises the external
 * controller heartbeat.
 *
 * The RX interrupt only transfers frames into CanRxQueueHandle. All message
 * decoding and application-state updates are performed in this task context.
 */

#include "task_can_rx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"

#include "main.h"

#include <stdio.h>

#include "control_mode_balance.h"
#include "system_state.h"
#include "can_protocol.h"

extern QueueHandle_t CanRxQueueHandle;

extern osEventFlagsId_t SystemEventGroupHandle;

extern FDCAN_HandleTypeDef hfdcan1;

extern osMessageQueueId_t StateRequestQueueHandle;

/*
 * Convert the STM32 FDCAN DLC field into the actual payload length.
 *
 * The application CAN protocol uses the payload length in bytes, while
 * the HAL receive header stores the encoded FDCAN DLC value.
 */
static uint8_t fdcan_dlc_to_bytes(uint32_t dlc_field)
{
    switch (dlc_field)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;

        default:
            return 8;
    }
}

/**
 * @brief Handle a new FDCAN FIFO0 message.
 *
 * The interrupt callback performs only the minimum work required to capture
 * the received frame and enqueue it for task-level processing.
 *
 * Keeping message decoding outside the ISR prevents CAN processing from
 * extending interrupt latency and allows the application logic to run in
 * normal FreeRTOS task context.
 *
 * @param hfdcan FDCAN peripheral handle.
 * @param RxFifo0ITs FIFO0 interrupt status flags.
 */
void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1)
        return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    FDCAN_RxHeaderTypeDef rxHeader;
    can_frame_t frame;

    if (HAL_FDCAN_GetRxMessage(
            hfdcan,
            FDCAN_RX_FIFO0,
            &rxHeader,
            frame.data) != HAL_OK)
    {
        return;
    }

    frame.id  = rxHeader.Identifier;
    frame.dlc = fdcan_dlc_to_bytes(rxHeader.DataLength);

    /*
     * Transfer ownership of the received frame from ISR context to
     * Task_CAN_RX. If a higher-priority task is unblocked by the queue
     * operation, request an immediate context switch after the ISR exits.
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(
        CanRxQueueHandle,
        &frame,
        &xHigherPriorityTaskWoken
    );

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void StartTaskCanRx(void *argument)
{
    (void)argument;

    can_frame_t frame;

    /*
     * The heartbeat timer is initialized when the task starts.
     * A timeout is therefore detected if no valid heartbeat is received
     * within CAN_HEARTBEAT_TIMEOUT_MS after startup.
     */
    TickType_t last_heartbeat_tick = xTaskGetTickCount();

    /*
     * Prevent repeated fault events while the heartbeat remains absent.
     * A new fault event is generated only when entering the fault condition.
     */
    bool fault_latched = false;

    setpoint_t sp;

    for (;;)
    {
        /*
         * The bounded queue wait keeps the task responsive even when no CAN
         * frame arrives. This allows heartbeat supervision and watchdog
         * reporting to continue independently of CAN traffic.
         */
        FDCAN_ProtocolStatusTypeDef ps;

        HAL_FDCAN_GetProtocolStatus(
            &hfdcan1,
            &ps
        );

        static uint32_t s_busoff_count = 0;

        if (ps.BusOff)
        {
            s_busoff_count++;

            /*
             * Restart the FDCAN peripheral after a bus-off condition.
             * The counter is retained for diagnostics without changing
             * the recovery behavior.
             */
            HAL_FDCAN_Stop(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
        }

        BaseType_t got = xQueueReceive(
            CanRxQueueHandle,
            &frame,
            pdMS_TO_TICKS(50)
        );

        if (got == pdTRUE)
        {
            switch (frame.id)
            {
                case CAN_ID_BALL_POS:
                    if (frame.dlc >= 4)
                    {
                        int16_t x = can_rd_i16le(&frame.data[0]);
                        int16_t y = can_rd_i16le(&frame.data[2]);

                        /*
                         * A valid ball-position frame also confirms that the
                         * external vision/controller path is actively sending
                         * data, so it refreshes the camera heartbeat.
                         */
                        ball_state_write_pos(
                            system_state_get_ball_ptr(),
                            x,
                            y
                        );

                        camera_heartbeat_mark();
                    }
                    break;

                case CAN_ID_BALL_VEL:
                    if (frame.dlc >= 4)
                    {
                        int16_t vx = can_rd_i16le(&frame.data[0]);
                        int16_t vy = can_rd_i16le(&frame.data[2]);

                        ball_state_write_vel(
                            system_state_get_ball_ptr(),
                            vx,
                            vy
                        );

                        /*
                         * Ball velocity is also produced by the external
                         * vision path, so receiving it confirms controller
                         * activity and refreshes the camera heartbeat.
                         */
                        camera_heartbeat_mark();
                    }
                    break;

                case CAN_ID_BALL_STATE:
                    if (frame.dlc >= 1)
                    {
                        ball_state_write_detected(
                            system_state_get_ball_ptr(),
                            frame.data[0]
                        );

                        /*
                         * Balance mode uses the decoded detection state to
                         * decide whether a valid ball is currently available.
                         */
                        control_mode_balance_set_ball_on(
                            frame.data[0] != 0
                        );
                    }
                    break;

                case CAN_ID_SERVO_CALIB:
                    /*
                     * Servo calibration remains owned by the internal
                     * calibration procedure. This CAN message is decoded
                     * only as a future override channel and must not directly
                     * overwrite active calibration data.
                     */
                    printf(
                        "RX 0x203: dlc=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        frame.dlc,
                        frame.data[0],
                        frame.data[1],
                        frame.data[2],
                        frame.data[3],
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7]
                    );

                    if (frame.dlc >= 6)
                    {
                        uint16_t s1c = can_rd_u16le(&frame.data[0]);
                        uint16_t s2c = can_rd_u16le(&frame.data[2]);
                        uint16_t s3c = can_rd_u16le(&frame.data[4]);

                        /*
                         * Keep the decoded values local until an explicit
                         * calibration-override contract is implemented.
                         */
                        (void)s1c;
                        (void)s2c;
                        (void)s3c;
                    }
                    break;

                case CAN_ID_ATTITUDE_DESIRED:
                    if (frame.dlc >= 6)
                    {
                        float roll_d =
                            can_rd_i16le(&frame.data[0]) / 100.0f;

                        float pitch_d =
                            can_rd_i16le(&frame.data[2]) / 100.0f;

                        float height_d =
                            (float)can_rd_i16le(&frame.data[4]);

                        /*
                         * Update only the fields carried by this CAN message.
                         * Read-modify-write preserves other setpoint fields
                         * such as mode and ball-position targets.
                         *
                         * If the shared setpoint cannot be acquired, discard
                         * the update rather than publishing a partial command.
                         */
                        if (setpoint_get(&sp))
                        {
                            sp.Roll_d   = roll_d;
                            sp.Pitch_d  = pitch_d;
                            sp.Height_d = height_d;

                            setpoint_set(&sp);
                        }
                    }
                    break;

                case CAN_ID_HEARTBEAT_RX:
                    /*
                     * A valid heartbeat refreshes the external-controller
                     * supervision timer.
                     */
                    last_heartbeat_tick = xTaskGetTickCount();

                    if (fault_latched)
                    {
                        /*
                         * The current implementation clears the CAN fault
                         * automatically when heartbeat communication resumes.
                         *
                         * This keeps the recovery behavior symmetric with
                         * the heartbeat timeout detection below.
                         */
                        osEventFlagsClear(
                            SystemEventGroupHandle,
                            EVT_BIT_FAULT
                        );

                        fault_latched = false;
                    }
                    break;

                default:
                    /*
                     * Unknown CAN identifiers are ignored.
                     * They do not represent a local application fault because
                     * the CAN bus may contain traffic owned by other nodes.
                     */
                    break;
            }
        }

        /*
         * Heartbeat supervision runs on every task iteration, including
         * iterations where no CAN frame was received.
         *
         * The bounded queue timeout guarantees that this check cannot be
         * blocked indefinitely by an idle CAN bus.
         */
        if ((xTaskGetTickCount() - last_heartbeat_tick) >
            pdMS_TO_TICKS(CAN_HEARTBEAT_TIMEOUT_MS))
        {
            if (!fault_latched)
            {
                /*
                 * Publish the fault through the event flags used by the
                 * runtime safety path.
                 */
                osEventFlagsSet(
                    SystemEventGroupHandle,
                    EVT_BIT_FAULT
                );

                fault_latched = true;

                printf("FAULT SET - mat heartbeat!\r\n");

                /*
                 * Request the system state transition separately from the
                 * event flag. The queue uses a non-blocking send so a full
                 * state-request queue cannot stall CAN reception.
                 */
                state_event_t fault_evt = EVT_FAULT_DETECTED;

                osMessageQueuePut(
                    StateRequestQueueHandle,
                    &fault_evt,
                    0,
                    0
                );
            }
        }

        /*
         * Report task liveness after completing the CAN processing and
         * heartbeat supervision for this cycle.
         */
        task_alive_mark(ALIVE_BIT_CAN_RX);
    }
}

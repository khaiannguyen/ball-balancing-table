/**
 * @file    task_can_tx.c
 * @brief   Periodic CAN transmission task.
 *
 * Publishes STM32 system status, sensor data, actuator feedback, desired
 * setpoints, and heartbeat information to the CAN bus.
 *
 * The task uses Classic CAN frames with message-specific payload lengths.
 * CAN transmission is non-blocking so external bus conditions cannot stall
 * the real-time task.
 */

#include "task_can_tx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"
#include "system_state.h"
#include "can_protocol.h"

#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern osEventFlagsId_t    SystemEventGroupHandle;

/*
 * Convert an application payload length into the corresponding STM32 FDCAN
 * DLC encoding.
 *
 * Using the actual payload length for each message avoids transmitting
 * unnecessary bytes on the CAN bus.
 */
static uint32_t dlc_to_fdcan(uint8_t n)
{
    switch (n)
    {
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

/*
 * Read an event flag without clearing it.
 *
 * Multiple tasks may consume the same event group, so status reporting must
 * not modify the flag state observed by other consumers.
 */
static bool event_flag_is_set(osEventFlagsId_t h, uint32_t bit)
{
    uint32_t r =
        osEventFlagsWait(
            h,
            bit,
            osFlagsWaitAny | osFlagsNoClear,
            0
        );

    if ((r & osFlagsError) != 0)
        return false;

    return (r & bit) != 0;
}

/*
 * Queue one Classic CAN frame for transmission.
 *
 * The operation is intentionally non-blocking. If the FDCAN transmit queue
 * cannot accept the frame, the current sample is dropped rather than
 * blocking the periodic CAN task.
 */
static bool can_send(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef h = {0};

    h.Identifier          = id;
    h.IdType              = FDCAN_STANDARD_ID;
    h.TxFrameType         = FDCAN_DATA_FRAME;
    h.DataLength          = dlc_to_fdcan(dlc);
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch       = FDCAN_BRS_OFF;
    h.FDFormat            = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h.MessageMarker       = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &h,
        (uint8_t *)data
    ) == HAL_OK;
}

void StartTaskCanTx(void *argument)
{
    (void)argument;

    printf("StartTaskCanTx STARTED\r\n");

    TickType_t lastWake = xTaskGetTickCount();

    /*
     * Run the CAN publisher at 100 Hz.
     *
     * vTaskDelayUntil() keeps the transmission schedule periodic and avoids
     * accumulating timing drift between iterations.
     */
    const TickType_t period = pdMS_TO_TICKS(10);

    uint8_t heartbeat_counter = 0;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, period);

        uint8_t buf[8];

        /*
         * 0x100 ATTITUDE:
         *   Roll, Pitch  : signed int16, scale 0.01 deg
         *   Height       : signed int16, millimeters
         *
         * The current height field is sourced from the active setpoint because
         * the system does not currently provide a direct height measurement.
         */
        {
            float roll;
            float pitch;
            float vroll;
            float vpitch;

            imu_state_read(
                system_state_get_imu_ptr(),
                &roll,
                &pitch,
                &vroll,
                &vpitch
            );

            setpoint_t sp;
            float height = 0.0f;

            if (setpoint_get(&sp))
                height = sp.Height_d;

            can_wr_i16le(
                &buf[0],
                (int16_t)(roll * 100.0f)
            );

            can_wr_i16le(
                &buf[2],
                (int16_t)(pitch * 100.0f)
            );

            can_wr_i16le(
                &buf[4],
                (int16_t)height
            );

            can_send(
                CAN_ID_ATTITUDE,
                6,
                buf
            );

            /*
             * 0x101 RATE:
             *   Roll rate  : signed int16, scale 0.01 deg/s
             *   Pitch rate : signed int16, scale 0.01 deg/s
             */
            can_wr_i16le(
                &buf[0],
                (int16_t)(vroll * 100.0f)
            );

            can_wr_i16le(
                &buf[2],
                (int16_t)(vpitch * 100.0f)
            );

            can_send(
                CAN_ID_RATE,
                4,
                buf
            );
        }

        /*
         * 0x102 SERVO_POS:
         *   S1, S2, S3 : unsigned int16, microseconds
         *
         * Reports the actuator command values currently stored in the
         * shared actuator state.
         */
        {
            int32_t s1;
            int32_t s2;
            int32_t s3;

            actuator_state_read(
                system_state_get_actuator_ptr(),
                &s1,
                &s2,
                &s3
            );

            can_wr_u16le(
                &buf[0],
                (uint16_t)s1
            );

            can_wr_u16le(
                &buf[2],
                (uint16_t)s2
            );

            can_wr_u16le(
                &buf[4],
                (uint16_t)s3
            );

            can_send(
                CAN_ID_SERVO_POS,
                6,
                buf
            );
        }

        /*
         * 0x103 ROBOT_STATE:
         *   Byte 0 : current control mode
         *   Byte 1 : RUN, FAULT, and camera-status flags
         *
         * Event flags are observed without clearing them so the status
         * publisher cannot interfere with safety or control tasks.
         */
        {
            setpoint_t sp;
            uint8_t mode = 0;

            if (setpoint_get(&sp))
                mode = sp.mode;

            uint8_t bits = 0;

            if (system_state_get() == STATE_RUN)
                bits |= ROBOT_STATE_BIT_RUN;

            if (event_flag_is_set(
                    SystemEventGroupHandle,
                    EVT_BIT_FAULT))
            {
                bits |= ROBOT_STATE_BIT_FAULT;
            }

            if (camera_state_is_ok())
                bits |= ROBOT_STATE_BIT_CAM_OK;

            buf[0] = mode;
            buf[1] = bits;

            can_send(
                CAN_ID_ROBOT_STATE,
                2,
                buf
            );
        }

        /*
         * 0x104 BALL_DESIRED:
         *   Ball X target : signed int16, millimeters
         *   Ball Y target : signed int16, millimeters
         *   Height target : reserved and currently transmitted as zero
         *
         * The current setpoint structure does not contain a ball-height
         * target, so the reserved field remains zero until the protocol and
         * setpoint model are extended together.
         */
        {
            setpoint_t sp;

            if (setpoint_get(&sp))
            {
                can_wr_i16le(
                    &buf[0],
                    (int16_t)sp.Ballx_d
                );

                can_wr_i16le(
                    &buf[2],
                    (int16_t)sp.Bally_d
                );

                can_wr_i16le(
                    &buf[4],
                    0
                );

                can_send(
                    CAN_ID_BALL_DESIRED,
                    6,
                    buf
                );
            }
        }

        /*
         * 0x1FF HEARTBEAT:
         *
         * The counter provides a monotonically incrementing activity marker
         * for the external CAN node. uint8_t overflow intentionally wraps
         * the value from 255 back to 0.
         */
        buf[0] = heartbeat_counter++;

        can_send(
            CAN_ID_HEARTBEAT_TX,
            1,
            buf
        );

        /*
         * Task_CAN_TX is intentionally excluded from the expected watchdog
         * alive mask. The watchdog monitors the tasks that are required for
         * control and CAN reception, while this publisher task is not part
         * of that liveness contract.
         */
    }
}

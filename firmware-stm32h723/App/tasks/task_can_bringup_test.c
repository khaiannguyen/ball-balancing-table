/**
 * @file    task_can_bringup_test.c
 * @brief   CAN bring-up traffic generator for integration testing.
 *
 * Generates deterministic CAN frames so the CAN RX path and heartbeat
 * failsafe can be validated before the external Jetson/ESP32 controller
 * is available.
 *
 * This task is intended for bring-up and integration testing only.
 */

#include "task_can_bringup_test.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "can_protocol.h"
#include "system_state.h"

extern FDCAN_HandleTypeDef hfdcan1;

/**
 * @brief Transmit one simulated CAN frame.
 *
 * Builds a Classic CAN standard frame and places it into the FDCAN
 * transmit FIFO queue.
 *
 * @param id   Standard CAN identifier.
 * @param dlc  Payload length in bytes.
 * @param data Payload buffer.
 *
 * @return true when the frame is accepted by the FDCAN transmit queue.
 */
static bool send_fake(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    FDCAN_TxHeaderTypeDef h = {0};

    h.Identifier         = id;
    h.IdType             = FDCAN_STANDARD_ID;
    h.TxFrameType        = FDCAN_DATA_FRAME;
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch      = FDCAN_BRS_OFF;
    h.FDFormat           = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    /*
     * Map the application payload length to the STM32 FDCAN DLC encoding.
     * Only the payload sizes used by the bring-up test are handled
     * explicitly; unsupported sizes fall back to an 8-byte frame.
     */
    switch (dlc)
    {
        case 1:
            h.DataLength = FDCAN_DLC_BYTES_1;
            break;

        case 4:
            h.DataLength = FDCAN_DLC_BYTES_4;
            break;

        case 6:
            h.DataLength = FDCAN_DLC_BYTES_6;
            break;

        default:
            h.DataLength = FDCAN_DLC_BYTES_8;
            break;
    }

    return HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan1,
        &h,
        (uint8_t *)data
    ) == HAL_OK;
}

void StartTaskCanBringupTest(void *argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();
    uint32_t cycle = 0;

    /*
     * Set this flag through the debugger to stop heartbeat transmission.
     * This allows the CAN watchdog/failsafe path to be tested without
     * changing the application logic.
     */
    bool stop_heartbeat = false;

    for (;;)
    {
        /*
         * Generate all test traffic from a fixed 100 ms task period.
         * Individual CAN messages use integer cycle divisors to establish
         * deterministic transmission intervals.
         */
        vTaskDelayUntil(
            &lastWake,
            pdMS_TO_TICKS(100)
        );

        cycle++;

        /*
         * Send a deterministic ATTITUDE_DESIRED frame every 500 ms.
         *
         * Fixed values make the payload easy to verify on a Logic Analyzer:
         *
         *   Roll_d   =  12.34 deg -> 1234 -> D2 04
         *   Pitch_d  =  -5.00 deg -> -500 -> 0C FE
         *   Height_d = 150 mm     -> 150  -> 96 00
         *
         * The expected 6-byte payload is:
         *
         *   D2 04 0C FE 96 00
         */
        if (cycle % 5 == 0)
        {
            uint8_t buf[6];

            can_wr_i16le(&buf[0], 1234);
            can_wr_i16le(&buf[2], -500);
            can_wr_i16le(&buf[4], 150);

            send_fake(
                CAN_ID_ATTITUDE_DESIRED,
                6,
                buf
            );
        }

        /*
         * Send a deterministic BALL_POS frame every 700 ms.
         * The values provide a known signed little-endian payload for
         * verifying the ball-position decoding path.
         */
        if (cycle % 7 == 0)
        {
            uint8_t buf[4];

            can_wr_i16le(&buf[0], 100);
            can_wr_i16le(&buf[2], -50);

            send_fake(
                CAN_ID_BALL_POS,
                4,
                buf
            );
        }

        /*
         * Report a continuously detected ball at the same interval as
         * BALL_POS so the CAN RX path receives a consistent test state.
         */
        if (cycle % 7 == 0)
        {
            uint8_t buf[1] = {1};

            send_fake(
                CAN_ID_BALL_STATE,
                1,
                buf
            );
        }

        /*
         * Transmit the heartbeat continuously unless the test flag is set.
         *
         * Stopping this message allows the heartbeat timeout/failsafe path
         * to be verified from the CAN bus without requiring an external
         * controller.
         */
        if (!stop_heartbeat)
        {
            uint8_t buf[1] = {(uint8_t)cycle};

            send_fake(
                CAN_ID_HEARTBEAT_RX,
                1,
                buf
            );
        }

        /*
         * This task temporarily represents the control-loop alive signal
         * when it is used in place of the normal control-loop task.
         *
         * The watchdog requires this bit to be refreshed because the task
         * participates in the expected alive-mask while the bring-up test
         * is active.
         */
        task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
    }
}

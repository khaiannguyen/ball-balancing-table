/**
 * @file    task_watchdog.c
 * @brief   System watchdog supervision task.
 *
 * Supervises the liveness of safety-critical application tasks and refreshes
 * the independent watchdog only when all required tasks have reported alive
 * during the current supervision window.
 *
 * A missing alive bit intentionally prevents watchdog refresh and allows the
 * hardware IWDG to reset the MCU.
 */

#include "task_watchdog.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "system_state.h"

extern IWDG_HandleTypeDef hiwdg1;

void StartTaskWatchdog(void *argument)
{
    (void)argument;

    TickType_t lastWake =
        xTaskGetTickCount();

    /*
     * Run the watchdog supervision at 10 Hz.
     *
     * Each supervision cycle represents one liveness window for the
     * safety-critical tasks included in ALIVE_MASK_EXPECTED.
     */
    const TickType_t period =
        pdMS_TO_TICKS(100);

    for (;;)
    {
        vTaskDelayUntil(
            &lastWake,
            period
        );

        /*
         * Atomically capture and clear the current alive mask.
         *
         * Clearing the mask as part of the snapshot prevents a race between
         * reading the task liveness state and starting the next supervision
         * window.
         */
        uint32_t mask =
            task_alive_snapshot_and_clear();

        if (mask == ALIVE_MASK_EXPECTED)
        {
            /*
             * Every required task has completed at least one liveness
             * checkpoint during the supervision window.
             *
             * Only this condition permits the hardware watchdog to be
             * refreshed.
             */
            HAL_IWDG_Refresh(&hiwdg1);
        }

        /*
         * If any required task failed to report alive, deliberately do not
         * refresh the watchdog.
         *
         * The independent watchdog will expire and reset the MCU, providing
         * a hardware-enforced recovery path for task deadlock or scheduler
         * failure.
         *
         * Avoid logging from this path because diagnostic output could block
         * or delay the watchdog failure mechanism.
         */
    }
}

/**
 * @file    task_display.c
 * @brief   Display task and screen manager service.
 *
 * Initializes the TFT display and UI data, manages screen transitions,
 * and periodically refreshes the active screen at 25 Hz.
 *
 * The task also handles the boot-screen synchronization and fault-screen
 * transitions without participating in the system watchdog alive mask.
 */

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

extern osEventFlagsId_t SystemEventGroupHandle;

/*
 * The display task is intentionally excluded from the watchdog alive mask.
 *
 * The watchdog monitors only the tasks required for real-time control and
 * CAN communication. Adding the display task to that mask would make the
 * watchdog depend on a non-critical UI task.
 */
#define DISPLAY_PERIOD_MS (1000 / 25)

/*
 * Keep the boot screen visible for a minimum period after the display
 * initialization is complete.
 *
 * The task first waits for the boot-completion event and then compensates
 * only for the remaining display time. This avoids a fixed startup delay
 * while ensuring that boot diagnostics remain readable.
 */
#define BOOT_MIN_DISPLAY_MS 1500u

void StartTaskDisplay(void *argument)
{
    (void)argument;

    /*
     * Initialize the display and UI data before creating the initial screen.
     */
    TFT_Init();
    UiData_Init();

    /*
     * Initialize all screens that require button-label or screen-state
     * registration before the screen manager selects the boot screen.
     */
    ScreenHome_Get();
    ScreenCalibrate_Get();
    ScreenBalance_Get();
    ScreenPosition_Get();
    ScreenManual_Get();

    ScreenManager_Goto(ScreenBoot_Get());

    /*
     * The boot screen is now fully initialized and available for other tasks.
     * Signal this event before Task_ControlLoop attempts to publish boot logs.
     */
    osEventFlagsSet(
        SystemEventGroupHandle,
        EVT_BIT_TFT_READY
    );

    uint32_t bootScreenStartTick =
        osKernelGetTickCount();

    /*
     * Wait until the control task completes its initialization sequence.
     *
     * This event represents completion of the IMU, calibration, actuator,
     * and self-test initialization required before normal operation.
     */
    osEventFlagsWait(
        SystemEventGroupHandle,
        EVT_BIT_BOOT_DONE,
        osFlagsWaitAny,
        osWaitForever
    );

    /*
     * Compensate for any time difference between boot initialization and the
     * minimum display interval so the boot diagnostics remain visible.
     */
    uint32_t elapsedMs =
        (osKernelGetTickCount() - bootScreenStartTick)
        * 1000u
        / osKernelGetTickFreq();

    if (elapsedMs < BOOT_MIN_DISPLAY_MS)
    {
        osDelay(
            BOOT_MIN_DISPLAY_MS - elapsedMs
        );
    }

    /*
     * Normal runtime operation starts from the home screen after the boot
     * sequence has completed.
     */
    ScreenManager_Goto(ScreenHome_Get());

    bool wasFault = false;

    uint32_t lastWake =
        osKernelGetTickCount();

    for (;;)
    {
        /*
         * Synchronize the UI data from the shared system state before
         * rendering the active screen.
         *
         * This keeps displayed IMU, actuator, ball, and system-state values
         * consistent with the latest application state available to the UI.
         */
        UiData_SyncFromSystemState();

        bool fault =
            system_state_fault_is_set();

        /*
         * Enter the fault screen only on the fault rising edge.
         *
         * The current screen is pushed onto the screen stack so normal
         * operation can resume at the same screen after the fault clears.
         */
        if (fault && !wasFault)
        {
            ScreenManager_GotoAndRemember(
                ScreenFault_Get()
            );
        }
        /*
         * Restore the previous screen when the fault condition clears.
         */
        else if (!fault && wasFault)
        {
            ScreenManager_GoBack();
        }

        wasFault = fault;

        /*
         * Update the active screen once per display cycle.
         */
        ScreenManager_Update();

        /*
         * Maintain a fixed 25 Hz update schedule without accumulating
         * execution-time drift between cycles.
         */
        lastWake += DISPLAY_PERIOD_MS;

        osDelayUntil(lastWake);
    }
}

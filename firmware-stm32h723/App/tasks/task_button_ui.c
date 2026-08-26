#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "task_button_ui.h"
#include "buttons.h"
#include "screen.h"
#include "screen_gauge_common.h"
#include "system_state.h"
#include "task_state_machine.h"
#include "main.h"

extern osMessageQueueId_t StateRequestQueueHandle;

/*
 * Handle a long press of BTN1.
 *
 * BTN1 long press is used for the primary RUN/STOP action and for
 * acknowledging SAFE_MODE after the active fault condition has cleared.
 *
 * State transitions are requested through StateRequestQueueHandle.
 * Task_StateMachine remains the sole owner responsible for publishing
 * system-state changes.
 */
static void HandleBtn1Long(void)
{
    system_state_t cur = system_state_get();
    bool inFault = system_state_fault_is_set();

    if (cur == STATE_RUN)
    {
        state_event_t evt = EVT_BTN_STOP;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        /*
         * Keep the Fault screen visible when a fault is active.
         * The normal Stop overlay is only meaningful on the gauge screen.
         */
        if (!inFault)
        {
            ScreenGauge_SetStopped(true);
        }
    }
    else if (cur == STATE_READY)
    {
        state_event_t evt = EVT_BTN_RUN;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        /*
         * Clear the stopped overlay when returning to the RUN state.
         * Do not modify the Fault screen while a fault is active.
         */
        if (!inFault)
        {
            ScreenGauge_SetStopped(false);
        }
    }
    else if (cur == STATE_SAFE_MODE && !inFault)
    {
        /*
         * SAFE_MODE requires explicit user acknowledgement after the
         * fault condition has cleared. This prevents the system from
         * leaving SAFE_MODE automatically while the underlying fault
         * is still active.
         */
        state_event_t evt;

        evt = EVT_MANUAL_RESET_ACK;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        /*
         * These events currently complete the bring-up state transition
         * because dedicated Safety and Calibration tasks are not yet
         * responsible for generating them.
         *
         * Once those tasks are active, they should generate these events
         * and this task should only send EVT_MANUAL_RESET_ACK.
         */
        evt = EVT_SELFTEST_OK;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        evt = EVT_CALIB_DONE;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
    }

    /*
     * BTN1 long press has no state transition in INIT, CALIBRATION, SLEEP,
     * or ERROR. It also has no effect in SAFE_MODE while the fault remains
     * active.
     */
}

void StartTaskButtonUi(void *argument)
{
    (void)argument;

    button_event_t evt;

    for (;;)
    {
        osStatus_t st =
            osMessageQueueGet(
                ButtonEventQueueHandle,
                &evt,
                NULL,
                osWaitForever
            );

        if (st == osOK)
        {
            ButtonState_t s;
            s.event = BUTTON_EVENT_PRESS;

            bool forward = true;

            switch (evt.id)
            {
                case BTN_ID_1:
                    if (evt.type == BTN_EVT_SHORT_PRESS)
                    {
                        /*
                         * A short BTN1 press is forwarded to the screen
                         * manager as the ENTER action.
                         */
                        s.button = BUTTON_ENTER;
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
                    }
                    else if (evt.type == BTN_EVT_LONG_PRESS)
                    {
                        /*
                         * Long BTN1 is handled by the state-machine control
                         * path instead of the normal screen navigation path.
                         */
                        HandleBtn1Long();
                        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
                        forward = false;
                    }
                    else
                    {
                        /*
                         * Ignore unsupported BTN1 event types.
                         */
                        forward = false;
                    }
                    break;

                case BTN_ID_2:
                    s.button = BUTTON_UP;
                    break;

                case BTN_ID_3:
                    s.button = BUTTON_LEFT;
                    break;

                case BTN_ID_4:
                    s.button = BUTTON_DOWN;
                    break;

                case BTN_ID_5:
                    s.button = BUTTON_RIGHT;
                    break;

                case BTN_ID_6:
                    s.button = BUTTON_EXIT;
                    break;

                default:
                    /*
                     * Unknown button IDs must never reach the screen manager.
                     */
                    forward = false;
                    break;
            }

            /*
             * Forward normal navigation events to the screen manager.
             * State-machine events handled directly by HandleBtn1Long()
             * are intentionally excluded from this path.
             */
            if (forward)
            {
                ScreenManager_OnButton(s);
            }
        }

        /*
         * This task is event-driven and blocks on ButtonEventQueueHandle.
         * It is therefore not part of the periodic task-alive reporting path.
         */
    }
}

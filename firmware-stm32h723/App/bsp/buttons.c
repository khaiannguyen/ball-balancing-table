#include "buttons.h"
#include "main.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <stdbool.h>

/*
 * Button input handling is split into two execution contexts:
 *
 * 1. EXTI interrupt context:
 *      Detect the GPIO edge and reset the appropriate debounce timer.
 *
 * 2. FreeRTOS Timer Service Task context:
 *      Validate the input state and generate the button event.
 *
 * This separation keeps the EXTI handler short and ensures that
 * queue operations and timer callbacks are executed outside ISR context.
 *
 * The EXTI handler must therefore use only ISR-safe FreeRTOS APIs.
 * In particular, xTimerResetFromISR() is used instead of the normal
 * task-context timer API.
 */

/*
 * ButtonEventQueueHandle is created by CubeMX in main.c during
 * MX_FREERTOS_Init().
 *
 * This module uses the existing queue and does not create or redefine it.
 */

/*
 * Hardware mapping between button IDs and GPIO pins.
 *
 * All button inputs use GPIO pull-ups and are therefore active-low:
 *
 *     GPIO_PIN_RESET -> button pressed
 *     GPIO_PIN_SET   -> button released
 *
 * EXTI configuration:
 *
 *     BTN1 -> Rising + Falling
 *     BTN2 -> Falling
 *     BTN3 -> Falling
 *     BTN4 -> Falling
 *     BTN5 -> Falling
 *     BTN6 -> Falling
 *
 * BTN1 requires both edges because its short/long-press behavior
 * depends on detecting both press and release.
 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} ButtonPin_t;

static const ButtonPin_t s_buttonPin[BTN_ID_COUNT] =
{
    [BTN_ID_1] = { BTN1_GPIO_Port, BTN1_Pin },
    [BTN_ID_2] = { BTN2_GPIO_Port, BTN2_Pin },
    [BTN_ID_3] = { BTN3_GPIO_Port, BTN3_Pin },
    [BTN_ID_4] = { BTN4_GPIO_Port, BTN4_Pin },
    [BTN_ID_5] = { BTN5_GPIO_Port, BTN5_Pin },
    [BTN_ID_6] = { BTN6_GPIO_Port, BTN6_Pin },
};

/*
 * Each button has its own one-shot debounce timer.
 *
 * BTN1 also has a dedicated one-shot long-press timer.
 */
static TimerHandle_t s_debounceTimer[BTN_ID_COUNT];
static TimerHandle_t s_btn1LongPressTimer;

/*
 * Confirmed state after debounce validation.
 *
 * This state is used only for BTN1 because BTN1 has both press and
 * release EXTI events and therefore requires edge-independent state
 * tracking.
 */
static volatile bool s_confirmedPressed[BTN_ID_COUNT];

/*
 * Prevents BTN1 from generating both a long-press and a short-press
 * event for the same physical button action.
 */
static volatile bool s_btn1LongPressFired;

static inline bool ReadPressed(button_id_t id)
{
    return HAL_GPIO_ReadPin(s_buttonPin[id].port, s_buttonPin[id].pin) == GPIO_PIN_RESET;
}

static button_id_t PinToButtonId(uint16_t GPIO_Pin)
{
    for (int i = 0; i < BTN_ID_COUNT; i++)
    {
        if (s_buttonPin[i].pin == GPIO_Pin)
        {
            return (button_id_t)i;
        }
    }

    return BTN_ID_COUNT;
}

/*
 * @brief Generate the BTN1 long-press event.
 *
 * This callback executes in the FreeRTOS Timer Service Task context.
 * It is invoked only after BTN1 has remained pressed for the configured
 * long-press interval.
 */
static void Btn1LongPressCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    s_btn1LongPressFired = true;

    button_event_t evt =
    {
        .id   = BTN_ID_1,
        .type = BTN_EVT_LONG_PRESS
    };

    osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
}

/*
 * @brief Validate a short press for BTN2..BTN6.
 *
 * BTN2..BTN6 use falling-edge EXTI only. Since no release interrupt is
 * generated for these buttons, debounce is edge-based rather than
 * state-transition based.
 *
 * Every falling edge resets the one-shot timer. A button event is
 * generated only when the pin is still active-low after the debounce
 * interval.
 *
 * This prevents contact bounce from producing multiple button events
 * while avoiding the need to track a release state that cannot be
 * observed through EXTI.
 */
static void GenericDebounceCallback(TimerHandle_t xTimer)
{
    button_id_t id =
        (button_id_t)(uintptr_t)pvTimerGetTimerID(xTimer);

    if (ReadPressed(id))
    {
        button_event_t evt =
        {
            .id   = id,
            .type = BTN_EVT_SHORT_PRESS
        };

        osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
    }
}

/*
 * @brief Debounce and classify BTN1 press/release events.
 *
 * BTN1 supports both short-press and long-press detection.
 *
 * A confirmed press starts the long-press timer.
 * A confirmed release stops the timer and generates a short-press
 * event only if the long-press threshold has not already been reached.
 *
 * Once a long-press event has been generated, the subsequent release
 * must not generate an additional short-press event.
 */
static void Btn1DebounceCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    bool pressedNow = ReadPressed(BTN_ID_1);

    /*
     * Ignore the callback when the physical state has not changed
     * since the last confirmed state. This filters residual bounce
     * without generating duplicate events.
     */
    if (pressedNow == s_confirmedPressed[BTN_ID_1])
    {
        return;
    }

    s_confirmedPressed[BTN_ID_1] = pressedNow;

    if (pressedNow)
    {
        /*
         * A stable press has been confirmed.
         * Start the one-shot long-press timer.
         */
        s_btn1LongPressFired = false;

        xTimerStart(s_btn1LongPressTimer, 0);
    }
    else
    {
        /*
         * A stable release has been confirmed.
         * Cancel long-press detection if it is still pending.
         */
        xTimerStop(s_btn1LongPressTimer, 0);

        /*
         * If the long-press threshold was not reached, classify
         * this press/release sequence as a short press.
         */
        if (!s_btn1LongPressFired)
        {
            button_event_t evt =
            {
                .id   = BTN_ID_1,
                .type = BTN_EVT_SHORT_PRESS
            };

            osMessageQueuePut(ButtonEventQueueHandle, &evt, 0, 0);
        }

        /*
         * If a long-press event was already generated, the release
         * completes the same action and must not generate another event.
         */
    }
}

/*
 * @brief Initialize the button input processing subsystem.
 *
 * Creates one-shot debounce timers for all buttons and a dedicated
 * long-press timer for BTN1.
 *
 * The application event queue is intentionally not created here.
 * It is owned by the FreeRTOS initialization generated by CubeMX.
 */
void buttons_init(void)
{
    for (int i = 0; i < BTN_ID_COUNT; i++)
    {
        s_confirmedPressed[i] = false;

        TimerCallbackFunction_t cb =
            (i == BTN_ID_1)
                ? Btn1DebounceCallback
                : GenericDebounceCallback;

        s_debounceTimer[i] = xTimerCreate("btnDebounce", pdMS_TO_TICKS(BTN_DEBOUNCE_MS), pdFALSE, (void *)(uintptr_t)i, cb);
    }

    s_btn1LongPressFired = false;

    s_btn1LongPressTimer = xTimerCreate("btn1LongPress", pdMS_TO_TICKS(BTN_LONGPRESS_MS), pdFALSE, NULL, Btn1LongPressCallback);
}

/*
 * @brief Handle a GPIO EXTI event generated by a button.
 *
 * This function executes in interrupt context.
 *
 * The EXTI handler does not perform debounce processing directly.
 * Instead, it resets the corresponding one-shot timer. The timer
 * callback later validates the stable GPIO state in task context.
 *
 * Using xTimerResetFromISR() keeps the interrupt handler ISR-safe
 * and minimizes the amount of work performed at interrupt level.
 */
void buttons_exti_handler(uint16_t GPIO_Pin)
{
    button_id_t id = PinToButtonId(GPIO_Pin);

    if (id == BTN_ID_COUNT)
    {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTimerResetFromISR(s_debounceTimer[id], &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

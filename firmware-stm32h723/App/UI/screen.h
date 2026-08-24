#ifndef SCREEN_H
#define SCREEN_H

/*
 * ButtonState_t.
 *
 * This type was split out of the old button_service.h, which no
 * longer exists in the current architecture. The 6 physical buttons
 * and their button_id_t / button_event_t definitions (including
 * long-press support) are unchanged. Task_Button_UI is the only
 * place that maps those 6 physical buttons onto the 6-direction
 * ButtonState_t below before calling ScreenManager_OnButton().
 *
 * Button mapping:
 *
 *     BTN_ID_1 short press -> BUTTON_ENTER
 *     BTN_ID_1 long press  -> handled directly by Task_Button_UI as
 *                             an emergency RUN/STOP action; it does
 *                             not go through the screen manager
 *     BTN_ID_2              -> BUTTON_UP
 *     BTN_ID_3              -> BUTTON_LEFT
 *     BTN_ID_4              -> BUTTON_DOWN
 *     BTN_ID_5              -> BUTTON_RIGHT
 *     BTN_ID_6              -> BUTTON_EXIT
 */
typedef enum
{
    BUTTON_LEFT = 0,
    BUTTON_RIGHT,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_ENTER,
    BUTTON_EXIT
} ButtonId_t;

typedef enum
{
    BUTTON_EVENT_PRESS = 0,
    BUTTON_EVENT_RELEASE
} ButtonEventType_t;

typedef struct
{
    ButtonId_t        button;
    ButtonEventType_t event;
} ButtonState_t;

/*
 * Common interface implemented by every screen.
 */
typedef struct Screen
{
    void (*onEnter)(void);
    void (*onExit)(void);
    void (*update)(void);
    void (*onButton)(ButtonState_t evt);
} Screen_t;

/*
 * Screen manager.
 *
 * Navigation history is kept on a stack rather than in a single
 * "remembered previous screen" variable. This supports safe nesting:
 * for example, a Shutdown dialog is open, a Fault interrupts it, and
 * while the Fault screen is active the user long-presses RUN/STOP.
 * Each layer keeps its own slot on the stack, so GoBack() always
 * returns to the correct previous screen instead of being overwritten
 * or losing track of where it came from.
 */

void ScreenManager_Goto(const Screen_t *next);

/*
 * @brief Switch to another screen while remembering the current one.
 *
 * Pushes the current screen onto the navigation stack, then switches
 * to the given screen.
 */
void ScreenManager_GotoAndRemember(const Screen_t *next);

/*
 * @brief Return to the previous screen.
 *
 * Pops one screen off the top of the navigation stack and switches
 * back to it.
 */
void ScreenManager_GoBack(void);

void ScreenManager_Update(void);

void ScreenManager_OnButton(ButtonState_t evt);

/*
 * @brief Lock/unlock the recursive mutex protecting the TFT/screen state.
 *
 * Exposed so that screen_gauge_common.c can draw the STOP overlay on
 * top of the gauge circle from Task_Button_UI (which does not go
 * through ScreenManager_OnButton()) without racing against
 * Task_Display, which calls ScreenManager_Update() at 25 Hz. This is
 * the same reason this mutex was introduced in B5 - see the comment
 * in screen_manager.c for details.
 */
void ScreenManager_Lock(void);
void ScreenManager_Unlock(void);

#endif

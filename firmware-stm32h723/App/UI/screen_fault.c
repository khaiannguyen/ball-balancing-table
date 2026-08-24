#include "screen_fault.h"
#include "tft_service.h"
#include "system_state.h"

#define MSG_Y 90

/*
 * Fault message detail.
 *
 * system_state.h currently exposes only a single boolean,
 * EVT_BIT_FAULT, with no accompanying message text. An earlier draft
 * (ui_system_state.c) carried a message buffer alongside the fault
 * flag, but that has since been removed.
 *
 * To show the specific cause of a fault (e.g. distinguishing a lost
 * CAN heartbeat from another error), a small text buffer and a setter
 * function would need to be added to system_state.c - similar to the
 * old UiState_SetFault(bool, msg) - with Task_CAN_RX / Task_Safety
 * passing a description string when they set EVT_BIT_FAULT.
 *
 * For now this screen only shows a generic "FAULT" message, which is
 * enough to verify that the Fault screen's goto/goback behavior works
 * correctly.
 */

static void OnEnter(void)
{
    TFT_FillScreen(TFT_COLOR_BLACK);

    uint16_t tw, th;
    TFT_GetTextExtent("FAULT", 2, &tw, &th);
    TFT_DrawText((TFT_WIDTH - tw) / 2, 60, "FAULT", TFT_COLOR_RED, TFT_COLOR_BLACK, 2);

    TFT_DrawText(31, MSG_Y, "Check CAN and sensor signal", TFT_COLOR_YELLOW, TFT_COLOR_BLACK, 1);
}

static void Update(void)
{
    /* Static screen while a fault is active - no dynamic content to update yet */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;

    /*
     * Button input is intentionally ignored while a fault is active.
     * This screen exits automatically once the fault clears.
     */
}

static const Screen_t screenFault = {
    .onEnter  = OnEnter,
    .onExit   = NULL,
    .update   = Update,
    .onButton = OnButton,
};

const Screen_t* ScreenFault_Get(void)
{
    return &screenFault;
}

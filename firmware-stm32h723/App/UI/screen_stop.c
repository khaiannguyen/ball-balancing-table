#include "screen_stop.h"
#include "tft_service.h"

#define STOP_CIRCLE_R  60

static void OnEnter(void)
{
    TFT_FillScreen(TFT_COLOR_BLACK);

    uint16_t cx = TFT_WIDTH / 2;
    uint16_t cy = TFT_HEIGHT / 2;

    /* The background is already filled with a solid color,
     * so the fast circle renderer can be used to reduce
     * the number of SPI transactions to approximately
     * (2 * STOP_CIRCLE_R + 1) DMA transfers.
     */
    TFT_FillCircleFast(
        cx,
        cy,
        STOP_CIRCLE_R,
        TFT_COLOR_RED,
        TFT_COLOR_BLACK
    );

    uint16_t tw, th;

    TFT_GetTextExtent("STOP", 3, &tw, &th);

    TFT_DrawTextFast(
        cx - tw/2,
        cy - th/2,
        "stop",
        TFT_COLOR_WHITE,
        TFT_COLOR_RED,
        3
    );
}

static void Update(void)
{
    /* Static screen; no periodic redraw is required. */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;

    /* Button events are intentionally ignored here.
     * The STOP screen can only be exited by a second BTN1
     * long-press, which is handled directly by Task_Button_UI
     * instead of ScreenManager_OnButton().
     */
}

static const Screen_t screenStop = {
    .onEnter  = OnEnter,
    .onExit   = NULL,
    .update   = Update,
    .onButton = OnButton,
};

const Screen_t* ScreenStop_Get(void)
{
    return &screenStop;
}

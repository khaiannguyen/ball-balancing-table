#include "screen_stop.h"
#include "tft_service.h"

#define STOP_CIRCLE_R  60

static void OnEnter(void)
{
    TFT_FillScreen(TFT_COLOR_BLACK);

    uint16_t cx = TFT_WIDTH / 2;
    uint16_t cy = TFT_HEIGHT / 2;

    /* nen quanh hinh tron la den dac (vua FillScreen xong) nen dung
     * ban FAST duoc: giam tu hang ngan giao dich SPI xuong con
     * (2*STOP_CIRCLE_R+1) lan DMA */
    TFT_FillCircleFast(cx, cy, STOP_CIRCLE_R, TFT_COLOR_RED, TFT_COLOR_BLACK);

    uint16_t tw, th;
    TFT_GetTextExtent("STOP", 3, &tw, &th);
    TFT_DrawTextFast(cx - tw/2, cy - th/2, "stop", TFT_COLOR_WHITE, TFT_COLOR_RED, 3);
}

static void Update(void)
{
    /* màn hình tĩnh, không cần vẽ lại gì */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;
    /* Cố ý không xử lý gì ở đây - thoát STOP chỉ qua BTN1 long,
     * được Task_Button_UI xử lý trực tiếp (xem task_button_ui.c),
     * không đi qua ScreenManager_OnButton(). */
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

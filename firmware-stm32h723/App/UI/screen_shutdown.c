#include "screen_shutdown.h"
#include "screen.h"
#include "tft_service.h"
#include <string.h>

/* =========================================================
 * Layout đã chỉnh lại cho đúng 220x176 (trước đó nút NO ở
 * x=230..300 bị tràn ra ngoài màn hình thật, gây lỗi GRAM)
 * ========================================================= */

typedef enum
{
    SD_STATE_CONFIRM = 0,
    SD_STATE_LOGGING
} ShutdownState_t;

static ShutdownState_t state = SD_STATE_CONFIRM;

#define BTN_YES_CX   55
#define BTN_YES_CY   45
#define BTN_YES_R    24

#define BTN_NO_X0    110
#define BTN_NO_Y0    24
#define BTN_NO_X1    180
#define BTN_NO_Y1    66

#define LOG_BOX_X0   4
#define LOG_BOX_Y0   80
#define LOG_BOX_X1   (TFT_WIDTH-4)
#define LOG_BOX_Y1   (TFT_HEIGHT-4)

#define LOG_TEXT_X   (LOG_BOX_X0+6)
#define LOG_TEXT_Y0  (LOG_BOX_Y0+6)
#define LOG_LINE_H   12

#define SD_MAX_LINES 7
#define SD_LINE_LEN  34

static char logLines[SD_MAX_LINES][SD_LINE_LEN];
static uint8_t logCount = 0;


static void DrawConfirm(void)
{
	TFT_DrawTextFast(2, 2, "Back to HOME and shut down?", 0xFFFF, 0x0000, 1);
	//TFT_DrawTextFast(2, 12, "and shut down?", 0xFFFF, 0x0000, 1);

    TFT_FillCircleFast(BTN_YES_CX, BTN_YES_CY, BTN_YES_R, TFT_COLOR_RED, TFT_COLOR_BLACK);

    uint16_t tw, th;
    TFT_GetTextExtent("YES", 1, &tw, &th);
    TFT_DrawTextFast(
            BTN_YES_CX - tw/2,
            BTN_YES_CY - th/2,
            "YES", 0xFFFF, TFT_COLOR_RED, 1);

    TFT_FillRectangle(BTN_NO_X0, BTN_NO_Y0, BTN_NO_X1, BTN_NO_Y1, TFT_COLOR_GREEN);

    TFT_GetTextExtent("NO", 1, &tw, &th);
    TFT_DrawTextFast(
            BTN_NO_X0 + ((BTN_NO_X1-BTN_NO_X0)-tw)/2,
            BTN_NO_Y0 + ((BTN_NO_Y1-BTN_NO_Y0)-th)/2,
            "NO", 0xFFFF, TFT_COLOR_GREEN, 1);
}


static void DrawLogBox(void)
{
    TFT_DrawRectangle(LOG_BOX_X0, LOG_BOX_Y0, LOG_BOX_X1, LOG_BOX_Y1, 0xFFFF);

    for(uint8_t i=0; i<logCount; i++)
    {
        TFT_DrawTextFast(
                LOG_TEXT_X,
                LOG_TEXT_Y0 + i*LOG_LINE_H,
                logLines[i],
                0xFFE0,
                0x0000,
                1);
    }
}


static void OnEnter(void)
{
    state = SD_STATE_CONFIRM;
    logCount = 0;

    TFT_FillScreen(0x0000);

    DrawConfirm();
    DrawLogBox();
}

static void Update(void)
{
}

static void OnButton(ButtonState_t evt)
{
    if(evt.event != BUTTON_EVENT_PRESS)
    {
        return;
    }

    if(state == SD_STATE_CONFIRM)
    {
        if(evt.button == BUTTON_ENTER)
        {
            state = SD_STATE_LOGGING;
            /* TODO: gọi hàm thật báo task_safety/task_actuator
               bắt đầu quy trình về home + shutdown. */
        }
        else if(evt.button == BUTTON_EXIT)
        {
            ScreenManager_GoBack();
        }
    }
}

static const Screen_t screenShutdown = {
    .onEnter  = OnEnter,
    .onExit   = NULL,
    .update   = Update,
    .onButton = OnButton,
};

const Screen_t* ScreenShutdown_Get(void)
{
    return &screenShutdown;
}


void ScreenShutdown_AddLog(const char *line)
{
    if(logCount >= SD_MAX_LINES)
    {
        return;
    }

    strncpy(logLines[logCount], line, SD_LINE_LEN-1);
    logLines[logCount][SD_LINE_LEN-1] = '\0';

    TFT_DrawTextFast(
            LOG_TEXT_X,
            LOG_TEXT_Y0 + logCount*LOG_LINE_H,
            logLines[logCount],
            0xFFE0,
            0x0000,
            1);

    logCount++;
}

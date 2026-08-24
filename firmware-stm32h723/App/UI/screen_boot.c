#include "screen_boot.h"
#include "tft_service.h"
#include <string.h>
#include <stdio.h>

/*
 * Boot screen layout (matches the reference mockup):
 *
 *     - Title "PINGPONG-TABLE: BOOT........" at the top
 *     - Large white outlined box in the middle
 *     - Yellow boot log text inside the box
 */

#define BOOT_BOX_X0   10
#define BOOT_BOX_Y0   30
#define BOOT_BOX_X1   (TFT_WIDTH-10)
#define BOOT_BOX_Y1   (TFT_HEIGHT-10)

#define BOOT_TEXT_X   (BOOT_BOX_X0+10)
#define BOOT_TEXT_Y0  (BOOT_BOX_Y0+10)
#define BOOT_LINE_H   16     /* line spacing, depends on font scale */
#define BOOT_TEXT_SCALE 1

#define BOOT_MAX_LINES 12
#define BOOT_LINE_LEN  40

static char logLines[BOOT_MAX_LINES][BOOT_LINE_LEN];
static uint8_t logCount = 0;

/*
 * @brief Draw the static frame and title for the boot screen.
 *
 * Called once on screen entry. Also redraws any log lines already
 * collected, in case this screen is entered more than once.
 */
static void Draw(void)
{
    TFT_FillScreen(0x0000); /* black */

    TFT_DrawText(10, 10, "PINGPONG-TABLE: BOOT........",
            0xFFFF, 0x0000, 1);

    TFT_DrawRectangle(
            BOOT_BOX_X0, BOOT_BOX_Y0,
            BOOT_BOX_X1, BOOT_BOX_Y1,
            0xFFFF);

    for(uint8_t i=0; i<logCount; i++)
    {
        TFT_DrawText(
                BOOT_TEXT_X,
                BOOT_TEXT_Y0 + i*BOOT_LINE_H,
                logLines[i],
                0xFFE0 /* yellow */,
                0x0000,
                BOOT_TEXT_SCALE);
    }
}

static void OnEnter(void)
{
    Draw();
}

static void Update(void)
{
    /*
     * No periodic redraw needed here. Log lines are drawn immediately
     * as they arrive, from ScreenBoot_AddLog().
     */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;

    /*
     * This screen does not respond to button input. The transition
     * away from it (e.g. to screen_home) is triggered externally,
     * once boot completes - for example by task_ui, after it sees
     * enough "READY" log lines or a timeout elapses.
     */
}

static const Screen_t screenBoot = {
    .onEnter  = OnEnter,
    .onExit   = NULL,
    .update   = Update,
    .onButton = OnButton,
};

const Screen_t* ScreenBoot_Get(void)
{
    return &screenBoot;
}

/*
 * @brief Append a line to the boot log and draw it immediately.
 *
 * Can be called from anywhere that has new boot information to show -
 * a UART RX callback, init code, etc.
 *
 * Locking:
 *   As of B7, this function is called from Task_ControlLoop, which
 *   logs each IMU/calibration/servo init step. That is a different
 *   task from Task_Display, which is concurrently calling
 *   ScreenManager_Update() at 25 Hz. Drawing to the TFT here without
 *   holding the screen lock would reproduce the exact race condition
 *   that caused overlapping text back in B5 (see the comment in
 *   screen_manager.c). ScreenManager_Lock()/Unlock() use a recursive
 *   mutex, so this stays safe even if a nested call is added later.
 */
void ScreenBoot_AddLog(const char *line)
{
    ScreenManager_Lock();

    if(logCount >= BOOT_MAX_LINES)
    {
        /*
         * Log area is full: scroll up by dropping the oldest line and
         * shifting the rest up by one slot.
         */
        for(uint8_t i=1; i<BOOT_MAX_LINES; i++)
        {
            strncpy(logLines[i-1], logLines[i], BOOT_LINE_LEN);
        }
        logCount = BOOT_MAX_LINES-1;

        /* Redraw the whole log area since every line just shifted */
        TFT_FillRectangle(
                BOOT_BOX_X0+1, BOOT_BOX_Y0+1,
                BOOT_BOX_X1-1, BOOT_BOX_Y1-1,
                0x0000);

        for(uint8_t i=0; i<logCount; i++)
        {
            TFT_DrawText(
                    BOOT_TEXT_X,
                    BOOT_TEXT_Y0 + i*BOOT_LINE_H,
                    logLines[i],
                    0xFFE0,
                    0x0000,
                    BOOT_TEXT_SCALE);
        }
    }

    strncpy(logLines[logCount], line, BOOT_LINE_LEN-1);
    logLines[logCount][BOOT_LINE_LEN-1] = '\0';

    /* Only the new line needs to be drawn; no full-screen redraw needed */
    TFT_DrawText(
            BOOT_TEXT_X,
            BOOT_TEXT_Y0 + logCount*BOOT_LINE_H,
            logLines[logCount],
            0xFFE0,
            0x0000,
            BOOT_TEXT_SCALE);

    logCount++;

    ScreenManager_Unlock();
}

void ScreenBoot_ClearLog(void)
{
    logCount = 0;
}

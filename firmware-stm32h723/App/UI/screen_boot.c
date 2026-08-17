#include "screen_boot.h"
#include "tft_service.h"
#include <string.h>
#include <stdio.h>

/* =========================================================
 * Layout (theo ảnh mẫu):
 * Title "PINGPONG-TABLE: BOOT........" trên cùng
 * Khung trắng to ở giữa, log màu vàng bên trong
 * ========================================================= */

#define BOOT_BOX_X0   10
#define BOOT_BOX_Y0   30
#define BOOT_BOX_X1   (TFT_WIDTH-10)
#define BOOT_BOX_Y1   (TFT_HEIGHT-10)

#define BOOT_TEXT_X   (BOOT_BOX_X0+10)
#define BOOT_TEXT_Y0  (BOOT_BOX_Y0+10)
#define BOOT_LINE_H   16     /* khoảng cách dòng, tuỳ scale font */
#define BOOT_TEXT_SCALE 1

#define BOOT_MAX_LINES 12
#define BOOT_LINE_LEN  40

static char logLines[BOOT_MAX_LINES][BOOT_LINE_LEN];
static uint8_t logCount = 0;


/*=========================================================
  Vẽ khung tĩnh + tiêu đề (gọi 1 lần khi vào screen)
=========================================================*/
static void Draw(void)
{
    TFT_FillScreen(0x0000); /* đen */

    TFT_DrawText(10, 10, "PINGPONG-TABLE: BOOT........",
            0xFFFF, 0x0000, 1);

    TFT_DrawRectangle(
            BOOT_BOX_X0, BOOT_BOX_Y0,
            BOOT_BOX_X1, BOOT_BOX_Y1,
            0xFFFF);

    /* vẽ lại các log đã có (nếu vào lại screen) */
    for(uint8_t i=0; i<logCount; i++)
    {
        TFT_DrawText(
                BOOT_TEXT_X,
                BOOT_TEXT_Y0 + i*BOOT_LINE_H,
                logLines[i],
                0xFFE0 /* vàng */,
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
    /* log được vẽ ngay khi ScreenBoot_AddLog() được gọi,
       không cần update lặp lại ở đây */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;
    /* Screen boot không xử lý nút bấm - tự động chuyển sang
       screen_home khi boot xong (gọi từ nơi khác, ví dụ task_ui
       sau khi nhận đủ log "READY" hoặc hết timeout) */
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


/*=========================================================
  API thêm log - gọi từ bất kỳ đâu (UART RX callback, init
  code...) khi STM32 có thông tin boot mới muốn hiển thị
=========================================================*/
void ScreenBoot_AddLog(const char *line)
{
    /* THÊM (B7): ScreenBoot_AddLog() giờ được gọi từ Task_ControlLoop (log
     * từng bước init IMU/calib/servo) - KHÁC task với Task_Display đang
     * chạy ScreenManager_Update() 25Hz. Vẽ thẳng TFT ở đây mà không khoá
     * sẽ lặp lại đúng race condition đã gây lỗi chồng chữ ở B5 (xem
     * comment trong screen_manager.c). ScreenManager_Lock()/Unlock() dùng
     * mutex recursive nên an toàn kể cả nếu sau này có nơi gọi lồng nhau. */
    ScreenManager_Lock();

    if(logCount >= BOOT_MAX_LINES)
    {
        /* hết chỗ: scroll lên - xoá dòng đầu, dồn lên */
        for(uint8_t i=1; i<BOOT_MAX_LINES; i++)
        {
            strncpy(logLines[i-1], logLines[i], BOOT_LINE_LEN);
        }
        logCount = BOOT_MAX_LINES-1;

        /* vẽ lại toàn bộ khung log vì đã scroll */
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

    /* vẽ ngay dòng mới, không cần vẽ lại cả màn hình */
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

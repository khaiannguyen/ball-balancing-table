#include "screen_fault.h"
#include "tft_service.h"
#include "system_state.h"

#define MSG_Y 90

/* =========================================================
 * system_state.h hiện CHỈ có 1 bit EVT_BIT_FAULT (bool), CHƯA có
 * message text kèm theo (khác bản nháp ui_system_state.c cũ mình
 * từng đưa msg buffer - nay bỏ). Nếu sau này bạn muốn hiện rõ
 * NGUYÊN NHÂN lỗi (vd "Mat heartbeat CAN" khác với 1 lỗi khác),
 * cần thêm 1 buffer nhỏ + hàm ghi trong system_state.c (tương tự
 * cách UiState_SetFault(bool,msg) bản cũ làm) rồi Task_CAN_RX/
 * Task_Safety gọi kèm chuỗi mô tả lúc set EVT_BIT_FAULT.
 * Hiện tại chỉ hiện chữ "FAULT" chung, đủ dùng để test Fault
 * screen goto/goback đúng chưa. ========================================================= */

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
    /* màn hình tĩnh trong lúc fault - chưa có message động để cập nhật */
}

static void OnButton(ButtonState_t evt)
{
    (void)evt;
    /* Cố ý không xử lý nút trong lúc Fault - thoát tự động khi hết lỗi */
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

#ifndef SCREEN_H
#define SCREEN_H

/* =========================================================
 * ButtonState_t - đã tách khỏi button_service.h cũ (không còn
 * tồn tại trong kiến trúc mới). button_id_t/button_event_t
 * (6 nút vật lý, có long-press) của B4 KHÔNG đổi - Task_Button_UI
 * là nơi duy nhất ánh xạ 6 nút đó -> ButtonState_t 6-hướng dưới đây
 * rồi mới gọi ScreenManager_OnButton().
 *
 * Ánh xạ (đã chốt cùng người dùng):
 *   BTN_ID_1 short -> BUTTON_ENTER
 *   BTN_ID_1 long  -> KHÔNG đi qua ScreenManager, xử lý riêng
 *                     trong Task_Button_UI (RUN/STOP khẩn cấp)
 *   BTN_ID_2       -> BUTTON_UP
 *   BTN_ID_3       -> BUTTON_LEFT
 *   BTN_ID_4       -> BUTTON_DOWN
 *   BTN_ID_5       -> BUTTON_RIGHT
 *   BTN_ID_6       -> BUTTON_EXIT
 * ========================================================= */

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

/* =========================================================
 * Interface chung cho mọi screen (không đổi so với bản gốc).
 * ========================================================= */
typedef struct Screen
{
    void (*onEnter)(void);
    void (*onExit)(void);
    void (*update)(void);
    void (*onButton)(ButtonState_t evt);
} Screen_t;

/* =========================================================
 * Screen manager - dùng STACK (không phải 1 biến "remembered"
 * duy nhất như bản gốc) để hỗ trợ lồng nhau an toàn:
 * ví dụ đang mở dialog Shutdown mà xảy ra Fault, rồi trong lúc
 * Fault người dùng long-press RUN/STOP -> mỗi lớp có chỗ nhớ
 * riêng trên stack, GoBack() luôn trả về đúng lớp ngay trước đó,
 * không bị ghi đè / mất dấu như khi chỉ có 1 biến remember.
 * ========================================================= */

void ScreenManager_Goto(const Screen_t *next);

/* Đẩy screen hiện tại vào stack rồi chuyển tới next */
void ScreenManager_GotoAndRemember(const Screen_t *next);

/* Lấy 1 screen ra khỏi đỉnh stack, quay lại đúng nơi đã rời đi */
void ScreenManager_GoBack(void);

void ScreenManager_Update(void);

void ScreenManager_OnButton(ButtonState_t evt);

/* THÊM: expose khoá mutex recursive đang bảo vệ TFT/screen (screen_manager.c)
 * ra ngoài, để screen_gauge_common.c có thể tự vẽ STOP overlay đè lên vòng
 * tròn TỪ Task_Button_UI (không đi qua ScreenManager_OnButton()) mà vẫn không
 * đụng độ với Task_Display đang gọi ScreenManager_Update() 25Hz - đúng lý do
 * mutex này được tạo ra ở B5 (xem comment trong screen_manager.c). */
void ScreenManager_Lock(void);
void ScreenManager_Unlock(void);

#endif

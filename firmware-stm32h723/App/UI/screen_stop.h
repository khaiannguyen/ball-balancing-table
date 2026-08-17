#ifndef SCREEN_STOP_H
#define SCREEN_STOP_H

#include "screen.h"

/* Screen tĩnh: chữ "STOP" trắng trong hình tròn đỏ, nền đen.
 * Hiện ra khi BTN1 long-press chuyển robot sang STOP (nút dừng
 * khẩn cấp). Không có logic onButton - thoát screen này CHỈ bằng
 * cách long-press BTN1 lần nữa (xử lý trực tiếp trong Task_Button_UI,
 * KHÔNG đi qua ScreenManager_OnButton) để tránh bất kỳ nút nào khác
 * vô tình "thoát STOP" ngoài đúng hành động đã định. */
const Screen_t* ScreenStop_Get(void);

#endif

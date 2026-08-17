#ifndef SCREEN_MANUAL_H
#define SCREEN_MANUAL_H
#include "screen.h"

/**
 * @brief Trả về Screen_t tĩnh cho Mode 4 - Manual, dùng chung layout với
 *        các gauge screen khác (screen_gauge_common.c) qua ScreenGauge_Get().
 *        Đăng ký hàm này vào nơi đang đăng ký ScreenHome_Get()/
 *        ScreenCalibrate_Get()/... (thường là screen_manager.c hoặc màn
 *        khởi tạo ban đầu) để LEFT/RIGHT ở Mode Position có thể duyệt tới.
 */
const Screen_t* ScreenManual_Get(void);

#endif /* SCREEN_MANUAL_H */

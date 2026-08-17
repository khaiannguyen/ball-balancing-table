#ifndef SCREEN_FAULT_H
#define SCREEN_FAULT_H

#include "screen.h"

/* Screen hiện khi system_state_fault_is_set() == true (Task_Display poll mỗi
 * vòng lặp). Tự động quay lại đúng screen trước đó khi hết lỗi -
 * xem Task_Display (task_display.c) - không cần bấm nút để thoát. */
const Screen_t* ScreenFault_Get(void);

#endif

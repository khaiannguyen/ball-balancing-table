#ifndef SCREEN_GAUGE_COMMON_H
#define SCREEN_GAUGE_COMMON_H

#include "screen.h"
#include <stdint.h>
#include <stdbool.h>
void ScreenGauge_SetStopped(bool stopped);
/* mode: 0=Home, 1=Calibrate, 2=Balance, 3=Position, 4=Manual (THÊM Giai đoạn 3) */
typedef enum
{
    MODE_HOME = 0,
    MODE_CALIBRATE,
    MODE_BALANCE,
    MODE_POSITION,
    MODE_MANUAL,     // THÊM Giai đoạn 3 (B6) - test tay servo/deadband/sweep
    MODE_COUNT
} GaugeMode_t;

/* bottomLabel: chữ trên nút xanh dưới cùng, ví dụ "0. Home"
 * Trả về con trỏ Screen_t tĩnh ứng với mode đó.
 */
const Screen_t* ScreenGauge_Get(uint8_t mode, const char *bottomLabel);

#endif

/* ---------------- screen_balance.c ---------------- */
#include "screen_balance.h"
#include "screen_gauge_common.h"

const Screen_t* ScreenBalance_Get(void)
{
    return ScreenGauge_Get(MODE_BALANCE, "2. Balance");
}

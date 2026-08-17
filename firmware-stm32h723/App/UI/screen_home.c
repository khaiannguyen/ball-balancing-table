/* ---------------- screen_home.c ---------------- */
#include "screen_home.h"
#include "screen_gauge_common.h"

const Screen_t* ScreenHome_Get(void)
{
    return ScreenGauge_Get(MODE_HOME, "0. Home");
}

/* ---------------- screen_position.c ---------------- */
#include "screen_position.h"
#include "screen_gauge_common.h"

const Screen_t* ScreenPosition_Get(void)
{
    return ScreenGauge_Get(MODE_POSITION, "3. Position");
}

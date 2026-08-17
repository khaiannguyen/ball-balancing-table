/* ---------------- screen_calibrate.c  ---------------- */
#include "screen_calibrate.h"
#include "screen_gauge_common.h"

const Screen_t* ScreenCalibrate_Get(void)
{
    return ScreenGauge_Get(MODE_CALIBRATE, "1. Calibrate");
}

#include "screen_manual.h"
#include "screen_gauge_common.h"

/**
 * @brief Wrapper mỏng, theo ĐÚNG pattern suy đoán từ ScreenGauge_Get() -
 *        CẦN ĐỐI CHIẾU lại với screen_calibrate.c/screen_home.c thật của
 *        bạn (chưa có trong context) để khớp tên hàm/label nếu khác quy ước
 *        này (vd nếu screen_calibrate.c dùng tên khác ScreenCalibrate_Get()).
 */
const Screen_t* ScreenManual_Get(void)
{
    return ScreenGauge_Get(MODE_MANUAL, "4. Manual");
}

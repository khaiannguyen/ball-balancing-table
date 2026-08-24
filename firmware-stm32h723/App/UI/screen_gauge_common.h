#ifndef SCREEN_GAUGE_COMMON_H
#define SCREEN_GAUGE_COMMON_H

#include "screen.h"
#include <stdint.h>
#include <stdbool.h>

void ScreenGauge_SetStopped(bool stopped);

/*
 * Gauge screen modes.
 *
 * MODE_MANUAL was added in Stage 3 for manually testing the servos
 * (deadband / step / sweep) directly from the UI.
 */
typedef enum
{
    MODE_HOME = 0,
    MODE_CALIBRATE,
    MODE_BALANCE,
    MODE_POSITION,
    MODE_MANUAL,     /* Stage 3 (B6) - manual servo deadband/step/sweep test */
    MODE_COUNT
} GaugeMode_t;

/*
 * @brief Get the gauge screen instance for a given mode.
 *
 * @param mode         One of the GaugeMode_t values.
 * @param bottomLabel  Text shown on the bottom blue mode button, e.g. "0. Home".
 *
 * @return Pointer to the static Screen_t instance for that mode.
 */
const Screen_t* ScreenGauge_Get(uint8_t mode, const char *bottomLabel);

#endif

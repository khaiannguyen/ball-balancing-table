#ifndef UI_DATA_H
#define UI_DATA_H

#include <stdint.h>
#include <stdbool.h>

/* Other tasks update the corresponding fields, while Task_Display
 * only reads them for rendering.
 *
 * The data follows a single-writer-per-field design, so no mutex
 * is required for UI access.
 */
typedef struct
{
    uint8_t mode;          /* 0=Home, 1=Calibrate, 2=Balance, 3=Position.
                            * Used for display only.
                            * The authoritative mode for control logic
                            * is setpoint_get().mode.
                            */

    bool robotRunning;     /* true=RUN, false=STOP.
                            * Reflects system_state_get() == STATE_RUN.
                            */

    float imuRoll;
    float imuPitch;

    bool ballOn;
    float ballX, ballY, ballVx, ballVy;

    bool cameraOk;

    /* Actual S1/S2/S3 servo commands in microseconds.
     * This replaces the previous motorOk[3] OK/ER status flags
     * and provides more useful information for debugging.
     */
    int16_t servoUs[3];

    char guideText[48];    /* Operating guidance text displayed by the STM32. */

} RobotUiData_t;

extern RobotUiData_t g_uiData;

void UiData_Init(void);

/* Synchronize UI data with the actual system state.
 *
 * This function should be called by Task_Display before
 * ScreenManager_Update() so that Roll/Pitch, S1-S3, and Ball
 * values shown by the UI reflect the latest system data.
 */
void UiData_SyncFromSystemState(void);

#endif

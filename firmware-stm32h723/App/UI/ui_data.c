#include "ui_data.h"
#include "system_state.h"
#include <string.h>

RobotUiData_t g_uiData;

void UiData_Init(void)
{
    memset(&g_uiData, 0, sizeof(g_uiData));
    strncpy(g_uiData.guideText, "Hold RED to STOP", sizeof(g_uiData.guideText) - 1);
}

void UiData_SyncFromSystemState(void)
{
    /* ---- IMU ---- */
    float roll, pitch, vroll, vpitch;
    imu_state_read(system_state_get_imu_ptr(), &roll, &pitch, &vroll, &vpitch);

    g_uiData.imuRoll  = roll;
    g_uiData.imuPitch = pitch;

    /* ---- Actuator ----
     * Use the actual S1/S2/S3 servo values instead of the previous
     * OK/ER status flags.
     */
    int32_t s1, s2, s3;
    actuator_state_read(system_state_get_actuator_ptr(), &s1, &s2, &s3);

    g_uiData.servoUs[0] = (int16_t)s1;
    g_uiData.servoUs[1] = (int16_t)s2;
    g_uiData.servoUs[2] = (int16_t)s3;

    /* ---- Ball ----
     * Ball position, velocity, and detection status are received
     * through CAN messages 0x200/0x201/0x202 by Task_CAN_RX.
     */
    int16_t bx, by, bvx, bvy;
    uint8_t detected;

    ball_state_read(
        system_state_get_ball_ptr(),
        &bx,
        &by,
        &bvx,
        &bvy,
        &detected
    );

    g_uiData.ballX  = (float)bx;
    g_uiData.ballY  = (float)by;
    g_uiData.ballVx = (float)bvx;
    g_uiData.ballVy = (float)bvy;
    g_uiData.ballOn = (detected != 0);

    /* ---- Mode ----
     * Read the current mode from setpoint_t. The UI writes the mode
     * when ENTER is committed.
     */
    setpoint_t sp;

    if (setpoint_get(&sp))
    {
        g_uiData.mode = sp.mode;
    }

    /* ---- RUN/STOP status ---- */
    g_uiData.robotRunning = (system_state_get() == STATE_RUN);

    /* cameraOk is true when Task_CAN_RX has received either
     * 0x200 BALL_POS or 0x201 BALL_VEL within the last 500 ms.
     * camera_heartbeat_mark() is called from task_can_rx.c.
     *
     * This is different from ballOn/ball_detected (0x202):
     * cameraOk indicates that the camera link is alive, not that
     * a ball is currently detected.
     */
    g_uiData.cameraOk = camera_state_is_ok();
}

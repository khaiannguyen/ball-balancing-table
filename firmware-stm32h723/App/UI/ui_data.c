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

    /* ---- Actuator (S1/S2/S3 thật, thay cho OK/ER cũ) ---- */
    int32_t s1, s2, s3;
    actuator_state_read(system_state_get_actuator_ptr(), &s1, &s2, &s3);
    g_uiData.servoUs[0] = (int16_t)s1;
    g_uiData.servoUs[1] = (int16_t)s2;
    g_uiData.servoUs[2] = (int16_t)s3;

    /* ---- Ball (từ CAN 0x200/0x201/0x202 qua Task_CAN_RX) ---- */
    int16_t bx, by, bvx, bvy;
    uint8_t detected;
    ball_state_read(system_state_get_ball_ptr(), &bx, &by, &bvx, &bvy, &detected);
    g_uiData.ballX   = (float)bx;
    g_uiData.ballY   = (float)by;
    g_uiData.ballVx  = (float)bvx;
    g_uiData.ballVy  = (float)bvy;
    g_uiData.ballOn  = (detected != 0);

    /* ---- Mode (từ setpoint_t.mode, do UI ghi lúc commit ENTER) ---- */
    setpoint_t sp;
    if (setpoint_get(&sp))
    {
        g_uiData.mode = sp.mode;
    }

    /* ---- RUN/STOP thật ---- */
    g_uiData.robotRunning = (system_state_get() == STATE_RUN);

    /* cameraOk: THẬT (đã nối B6) - true khi Task_CAN_RX nhận được
     * 0x200 BALL_POS hoặc 0x201 BALL_VEL trong 500ms gần nhất
     * (camera_heartbeat_mark() gọi từ task_can_rx.c). Khác với
     * ballOn/ball_detected (0x202) - "camera OK" chỉ là còn sống,
     * không có nghĩa là đang thấy bóng. */
    g_uiData.cameraOk = camera_state_is_ok();
}

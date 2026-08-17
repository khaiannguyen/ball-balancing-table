#ifndef UI_DATA_H
#define UI_DATA_H

#include <stdint.h>
#include <stdbool.h>

/* Các task khác (Task_IMU_Fusion, Task_CAN_RX, Task_ControlLoop...)
 * CẬP NHẬT, Task_Display CHỈ ĐỌC để vẽ - single-writer-per-field,
 * không cần mutex (đúng nguyên tắc mục 11.2/B.2.7). */
typedef struct
{
    uint8_t  mode;          /* 0=Home,1=Calibrate,2=Balance,3=Position - CHỈ để hiển thị,
                                nguồn "sự thật" của mode dùng cho logic là setpoint_get().mode */
    bool     robotRunning;  /* true=RUN, false=STOP - phản ánh system_state_get()==STATE_RUN */

    float    imuRoll;
    float    imuPitch;

    bool     ballOn;
    float    ballX, ballY, ballVx, ballVy;

    bool     cameraOk;

    /* THAY ĐỔI: trước là motorOk[3] (bool OK/ER) - giờ hiện trực tiếp
     * giá trị servo S1/S2/S3 thật (đơn vị µs, ghi bởi Task_ControlLoop
     * qua actuator_state_get(), xem mục B.2.7) để debug dễ hơn OK/ER. */
    int16_t  servoUs[3];

    char     guideText[48]; /* STM32 in hướng dẫn vận hành */
} RobotUiData_t;

extern RobotUiData_t g_uiData;

void UiData_Init(void);

/* THÊM cho B5: đọc giá trị THẬT từ system_state.h (seqlock/mutex) và
 * đổ vào g_uiData - gọi mỗi vòng Task_Display, TRƯỚC ScreenManager_Update().
 * Đây là chỗ trả lời "UI có hiện số thật không" - PHẢI gọi hàm này thì
 * Roll/Pitch/S1-3/Ball mới là số thật, không phải 0 tĩnh. */
void UiData_SyncFromSystemState(void);

#endif

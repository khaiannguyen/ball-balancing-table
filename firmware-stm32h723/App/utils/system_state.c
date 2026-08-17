#include "system_state.h"
#include "cmsis_os2.h"
#include <string.h>

/* Biến này CubeMX sinh trong freertos.c với tên đúng bằng tên bạn đặt ở tab Mutexes/Event Groups */
extern osMutexId_t SetpointMutexHandle;
extern osEventFlagsId_t SystemEventGroupHandle;   // nếu CubeMX sinh dạng EventFlags (CMSIS_V2)

static imu_state_t       g_imu;
static ball_state_t      g_ball;
static actuator_state_t  g_actuator;
static setpoint_t        g_setpoint;                 // bảo vệ bởi SetpointMutexHandle
static seqlock_t         g_state_lock;
static system_state_t    g_system_state = STATE_BOOT; // chỉ Task_StateMachine ghi
static volatile uint32_t g_task_alive_mask = 0;
static imu_raw_state_t   g_imu_raw;
static volatile uint32_t g_camera_last_rx_tick;   /* tick lúc nhận 0x200/0x201 gần nhất */
#define CAMERA_HEARTBEAT_TIMEOUT_MS  500u          /* cùng ngưỡng với 0x2FF HEARTBEAT_RX (mục 4.2) */

void system_state_init(void) {
    memset(&g_ball, 0, sizeof(g_ball));
    seqlock_init(&g_imu.lock);
    seqlock_init(&g_imu_raw.lock);
    seqlock_init(&g_ball.lock);
    seqlock_init(&g_actuator.lock);
    seqlock_init(&g_state_lock);
    memset(&g_setpoint, 0, sizeof(g_setpoint));
}

/* ---- IMU RAW (trước filter, ghi từ ISR) ---- */
void imu_raw_state_write(imu_raw_state_t *s, const int16_t *accel, const int16_t *gyro) {
    seqlock_write_begin(&s->lock);
    s->accel[0] = accel[0]; s->accel[1] = accel[1]; s->accel[2] = accel[2];
    s->gyro[0]  = gyro[0];  s->gyro[1]  = gyro[1];  s->gyro[2]  = gyro[2];
    seqlock_write_end(&s->lock);
}
void imu_raw_state_read(const imu_raw_state_t *s, int16_t *accel, int16_t *gyro) {
    uint32_t start;
    do {
        start = seqlock_read_begin(&s->lock);
        accel[0] = s->accel[0]; accel[1] = s->accel[1]; accel[2] = s->accel[2];
        gyro[0]  = s->gyro[0];  gyro[1]  = s->gyro[1];  gyro[2]  = s->gyro[2];
    } while (seqlock_read_retry(&s->lock, start));
}

/* ---- IMU RAW getter con trỏ (biến static, chỉ expose qua đây) ---- */
imu_raw_state_t *system_state_get_imu_raw_ptr(void) {
    return &g_imu_raw;
}

/* ---- Ball state (ghi từ Task_CAN_RX khi parse 0x200/0x201/0x202, mục 4.3) ---- */
/* Ghi 3 trường độc lập vì 3 frame CAN đến ở 3 thời điểm khác nhau (không cùng lúc);
   single-writer (chỉ Task_CAN_RX) nên không có race giữa các lần ghi này. */
void ball_state_write_pos(ball_state_t *s, int16_t x, int16_t y) {
    seqlock_write_begin(&s->lock);
    s->Ballx = x; s->Bally = y;
    seqlock_write_end(&s->lock);
}
void ball_state_write_vel(ball_state_t *s, int16_t vx, int16_t vy) {
    seqlock_write_begin(&s->lock);
    s->vBallx = vx; s->vBally = vy;
    seqlock_write_end(&s->lock);
}
void ball_state_write_detected(ball_state_t *s, uint8_t detected) {
    seqlock_write_begin(&s->lock);
    s->ball_detected = detected;
    seqlock_write_end(&s->lock);
}
void ball_state_read(const ball_state_t *s, int16_t *x, int16_t *y,
                      int16_t *vx, int16_t *vy, uint8_t *detected) {
    uint32_t start;
    do {
        start = seqlock_read_begin(&s->lock);
        *x = s->Ballx; *y = s->Bally;
        *vx = s->vBallx; *vy = s->vBally;
        *detected = s->ball_detected;
    } while (seqlock_read_retry(&s->lock, start));
}
ball_state_t *system_state_get_ball_ptr(void) {
    return &g_ball;
}

/* ---- Camera heartbeat (THÊM) ----
 * Gọi camera_heartbeat_mark() từ task_can_rx.c mỗi khi parse xong
 * CAN_ID_BALL_POS (0x200) HOẶC CAN_ID_BALL_VEL (0x201) - không cần
 * cả 2 cùng lúc, chỉ cần 1 trong 2 để biết Jetson còn sống. */
void camera_heartbeat_mark(void) {
    g_camera_last_rx_tick = osKernelGetTickCount();
}
bool camera_state_is_ok(void) {
    uint32_t now = osKernelGetTickCount();
    /* dùng osKernelGetTickFreq() thay vì pdMS_TO_TICKS() vì file này chỉ
     * include cmsis_os2.h (không có FreeRTOS.h) - quy đổi ms -> tick thủ công.
     * Trừ kiểu unsigned vẫn đúng kể cả khi tick counter wrap-around. */
    uint32_t timeout_ticks = (CAMERA_HEARTBEAT_TIMEOUT_MS * osKernelGetTickFreq()) / 1000u;
    return (uint32_t)(now - g_camera_last_rx_tick) < timeout_ticks;
}

/* ---- IMU ---- */
void imu_state_write(imu_state_t *s, float roll, float pitch, float vroll, float vpitch) {
    seqlock_write_begin(&s->lock);
    s->roll = roll; s->pitch = pitch; s->vroll = vroll; s->vpitch = vpitch;
    seqlock_write_end(&s->lock);
}
void imu_state_read(const imu_state_t *s, float *roll, float *pitch, float *vroll, float *vpitch) {
    uint32_t start;
    do {
        start = seqlock_read_begin(&s->lock);
        *roll = s->roll; *pitch = s->pitch; *vroll = s->vroll; *vpitch = s->vpitch;
    } while (seqlock_read_retry(&s->lock, start));
}

/* ---- IMU getter con trỏ (biến static, chỉ expose qua đây) ---- */
imu_state_t *system_state_get_imu_ptr(void) {
    return &g_imu;
}

/* ---- Actuator snapshot ---- */
void actuator_state_publish(actuator_state_t *s, int32_t s1, int32_t s2, int32_t s3) {
    seqlock_write_begin(&s->lock);
    s->S1 = s1; s->S2 = s2; s->S3 = s3;
    seqlock_write_end(&s->lock);
}
void actuator_state_read(const actuator_state_t *s, int32_t *s1, int32_t *s2, int32_t *s3) {
    uint32_t start;
    do {
        start = seqlock_read_begin(&s->lock);
        *s1 = s->S1; *s2 = s->S2; *s3 = s->S3;
    } while (seqlock_read_retry(&s->lock, start));
}

/* ---- Actuator getter con trỏ (biến static, chỉ expose qua đây) ---- */
actuator_state_t *system_state_get_actuator_ptr(void) {
    return &g_actuator;
}

/* ---- Setpoint (mutex, timeout ngắn tránh block control loop nếu lỡ gọi nhầm) ---- */
bool setpoint_get(setpoint_t *out) {
    if (osMutexAcquire(SetpointMutexHandle, 5) != osOK) return false;  // 5 ticks timeout
    *out = g_setpoint;
    osMutexRelease(SetpointMutexHandle);
    return true;
}
bool setpoint_set(const setpoint_t *in) {
    if (osMutexAcquire(SetpointMutexHandle, 5) != osOK) return false;
    g_setpoint = *in;
    osMutexRelease(SetpointMutexHandle);
    return true;
}

/* ---- System state ---- */
void system_state_publish(system_state_t s) {
    seqlock_write_begin(&g_state_lock);
    g_system_state = s;
    seqlock_write_end(&g_state_lock);
}
system_state_t system_state_get(void) {
    system_state_t v;
    uint32_t start;
    do {
        start = seqlock_read_begin(&g_state_lock);
        v = g_system_state;
    } while (seqlock_read_retry(&g_state_lock, start));
    return v;
}

/* ---- Task alive mask (Atomic, mục 3.6) ---- */
void task_alive_mark(uint32_t bit) {
    __atomic_fetch_or(&g_task_alive_mask, bit, __ATOMIC_SEQ_CST);
}
uint32_t task_alive_snapshot_and_clear(void) {
    return __atomic_exchange_n(&g_task_alive_mask, 0, __ATOMIC_SEQ_CST);
}

/* ---- Fault peek (THÊM cho B5) ----
 * Cùng pattern với event_flag_is_set() tĩnh trong task_can_tx.c:
 * osFlagsNoClear + timeout 0 -> đọc mà không xoá bit, không tranh
 * chấp với Task_ControlLoop/Task_CAN_TX cũng đang đọc EVT_BIT_FAULT. */
bool system_state_fault_is_set(void) {
    uint32_t r = osEventFlagsWait(SystemEventGroupHandle, EVT_BIT_FAULT,
                                   osFlagsWaitAny | osFlagsNoClear, 0);
    if ((r & osFlagsError) != 0) return false;   /* timeout 0 -> bit chưa set */
    return (r & EVT_BIT_FAULT) != 0;
}

/* system_state_request_run()/request_stop() (bản tạm dùng osEventFlags +
 * publish thẳng) ĐÃ BỊ XOÁ - task_state_machine.c thật đã có sẵn, là nơi
 * DUY NHẤT gọi system_state_publish(). Task_Button_UI giờ gửi state_event_t
 * (EVT_BTN_RUN/EVT_BTN_STOP) qua StateRequestQueueHandle, xem task_button_ui.c. */

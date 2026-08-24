#include "system_state.h"
#include "cmsis_os2.h"
#include <string.h>

/* CubeMX-generated mutex and event flag objects defined in freertos.c. */
extern osMutexId_t SetpointMutexHandle;
extern osEventFlagsId_t SystemEventGroupHandle;

static imu_state_t       g_imu;
static ball_state_t      g_ball;
static actuator_state_t  g_actuator;
static setpoint_t        g_setpoint;                 /* Protected by SetpointMutexHandle. */
static seqlock_t         g_state_lock;
static system_state_t    g_system_state = STATE_BOOT; /* Written by the state machine task. */
static volatile uint32_t g_task_alive_mask = 0;
static imu_raw_state_t   g_imu_raw;
static volatile uint32_t g_camera_last_rx_tick;      /* Tick when the latest 0x200/0x201 frame was received. */

#define CAMERA_HEARTBEAT_TIMEOUT_MS  500u
/* Must match the timeout used for CAN_ID_HEARTBEAT_RX (0x2FF). */

void system_state_init(void) {
    memset(&g_ball, 0, sizeof(g_ball));
    seqlock_init(&g_imu.lock);
    seqlock_init(&g_imu_raw.lock);
    seqlock_init(&g_ball.lock);
    seqlock_init(&g_actuator.lock);
    seqlock_init(&g_state_lock);
    memset(&g_setpoint, 0, sizeof(g_setpoint));
}

/* ---- Raw IMU state ----
 * Stores unfiltered accelerometer and gyroscope samples.
 * The write operation may be called from an ISR.
 */
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

/* Return a pointer to the internal raw IMU state object. */
imu_raw_state_t *system_state_get_imu_raw_ptr(void) {
    return &g_imu_raw;
}

/* ---- Ball state ----
 * Updated by Task_CAN_RX after parsing CAN frames
 * 0x200 (position), 0x201 (velocity), and 0x202 (detection state).
 *
 * The three fields are updated independently because the corresponding
 * CAN frames may arrive at different times. Task_CAN_RX is the single
 * writer, so there is no concurrent writer race between these updates.
 */
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

/* ---- Camera heartbeat ----
 * Called by Task_CAN_RX after successfully parsing either
 * CAN_ID_BALL_POS (0x200) or CAN_ID_BALL_VEL (0x201).
 *
 * Receiving either frame is sufficient to confirm that the
 * Jetson camera/control link is still active.
 */
void camera_heartbeat_mark(void) {
    g_camera_last_rx_tick = osKernelGetTickCount();
}

bool camera_state_is_ok(void) {
    uint32_t now = osKernelGetTickCount();

    /* Convert milliseconds to RTOS ticks using the CMSIS-RTOS tick frequency.
     * This file only includes cmsis_os2.h, so pdMS_TO_TICKS() is not available.
     *
     * Unsigned subtraction remains valid when the RTOS tick counter wraps around.
     */
    uint32_t timeout_ticks =
        (CAMERA_HEARTBEAT_TIMEOUT_MS * osKernelGetTickFreq()) / 1000u;

    return (uint32_t)(now - g_camera_last_rx_tick) < timeout_ticks;
}

/* ---- IMU state ---- */
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

/* Return a pointer to the internal filtered IMU state object. */
imu_state_t *system_state_get_imu_ptr(void) {
    return &g_imu;
}

/* ---- Actuator state snapshot ---- */
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

/* Return a pointer to the internal actuator state object. */
actuator_state_t *system_state_get_actuator_ptr(void) {
    return &g_actuator;
}

/* ---- Setpoint ----
 * Uses a mutex with a short timeout to avoid blocking the control loop
 * if this API is called from an unexpected execution context.
 */
bool setpoint_get(setpoint_t *out) {
    if (osMutexAcquire(SetpointMutexHandle, 5) != osOK) return false;  /* 5-tick timeout. */
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

/* ---- Task alive mask ----
 * Each task sets its assigned bit while running.
 * The watchdog/safety logic can atomically snapshot and clear
 * the mask to verify task liveness.
 */
void task_alive_mark(uint32_t bit) {
    __atomic_fetch_or(&g_task_alive_mask, bit, __ATOMIC_SEQ_CST);
}

uint32_t task_alive_snapshot_and_clear(void) {
    return __atomic_exchange_n(&g_task_alive_mask, 0, __ATOMIC_SEQ_CST);
}

/* ---- Fault status ----
 * Check whether EVT_BIT_FAULT is currently set without clearing it.
 *
 * osFlagsNoClear allows multiple tasks to observe the same fault flag
 * without consuming it. A zero timeout makes this a non-blocking query.
 */
bool system_state_fault_is_set(void) {
    uint32_t r = osEventFlagsWait(
        SystemEventGroupHandle,
        EVT_BIT_FAULT,
        osFlagsWaitAny | osFlagsNoClear,
        0
    );

    if ((r & osFlagsError) != 0) return false;   /* Non-blocking query; no fault flag available. */
    return (r & EVT_BIT_FAULT) != 0;
}

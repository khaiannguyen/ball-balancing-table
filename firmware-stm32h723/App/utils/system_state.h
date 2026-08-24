#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "seqlock.h"
#include "task_state_machine.h"

/* Operating modes used by the control loop. */
typedef enum {
    OPMODE_HOME     = 0,   // Move servos to neutral, then transition to READY
    OPMODE_CALIB    = 1,   // Automatic P-controller calibration
    OPMODE_BALANCE  = 2,   // Cascade Roll/Pitch/Height control
    OPMODE_POSITION = 3,   // Balance control with UI-selected ball target
    OPMODE_MANUAL   = 4,   // Manual servo testing and sweep logging
    OPMODE_COUNT_TOTAL      // Total number of operating modes
} operating_mode_t;


/* ---------- IMU state ---------- */

typedef struct {
    seqlock_t lock;
    float roll, pitch, vroll, vpitch;
} imu_state_t;

void imu_state_write(imu_state_t *s, float roll, float pitch, float vroll, float vpitch);
void imu_state_read(const imu_state_t *s, float *roll, float *pitch, float *vroll, float *vpitch);
imu_state_t *system_state_get_imu_ptr(void);


/* ---------- Raw IMU state ---------- */

typedef struct {
    seqlock_t lock;
    int16_t accel[3];
    int16_t gyro[3];
} imu_raw_state_t;

void imu_raw_state_write(imu_raw_state_t *s, const int16_t *accel, const int16_t *gyro);
void imu_raw_state_read(const imu_raw_state_t *s, int16_t *accel, int16_t *gyro);
imu_raw_state_t *system_state_get_imu_raw_ptr(void);


/* ---------- Ball state ---------- */

typedef struct {
    seqlock_t lock;
    int16_t Ballx, Bally;
    int16_t vBallx, vBally;
    uint8_t ball_detected;
} ball_state_t;

void ball_state_write_pos(ball_state_t *s, int16_t x, int16_t y);
void ball_state_write_vel(ball_state_t *s, int16_t vx, int16_t vy);
void ball_state_write_detected(ball_state_t *s, uint8_t detected);

void ball_state_read(
    const ball_state_t *s,
    int16_t *x,
    int16_t *y,
    int16_t *vx,
    int16_t *vy,
    uint8_t *detected
);

ball_state_t *system_state_get_ball_ptr(void);


/* ---------- Camera heartbeat ---------- */

/* Track the latest valid camera data received through CAN. */
void camera_heartbeat_mark(void);

/* Return true when the camera link is active. */
bool camera_state_is_ok(void);


/* ---------- Actuator state ---------- */

typedef struct {
    seqlock_t lock;
    int32_t S1, S2, S3;
} actuator_state_t;

void actuator_state_publish(
    actuator_state_t *s,
    int32_t s1,
    int32_t s2,
    int32_t s3
);

void actuator_state_read(
    const actuator_state_t *s,
    int32_t *s1,
    int32_t *s2,
    int32_t *s3
);

actuator_state_t *system_state_get_actuator_ptr(void);


/* ---------- Setpoint ---------- */

typedef struct {
    uint8_t mode;
    float Roll_d, Pitch_d, Height_d;
    float Ballx_d, Bally_d;
} setpoint_t;

bool setpoint_get(setpoint_t *out);
bool setpoint_set(const setpoint_t *in);


/* ---------- System state ---------- */

void system_state_publish(system_state_t s);
system_state_t system_state_get(void);


/* ---------- Task liveness ---------- */

#define ALIVE_BIT_CONTROL_LOOP   (1u << 0)
#define ALIVE_BIT_IMU_FUSION     (1u << 1)
#define ALIVE_BIT_CAN_RX         (1u << 2)
#define ALIVE_MASK_EXPECTED      ( \
    ALIVE_BIT_CONTROL_LOOP | \
    ALIVE_BIT_IMU_FUSION   | \
    ALIVE_BIT_CAN_RX)

void task_alive_mark(uint32_t bit);
uint32_t task_alive_snapshot_and_clear(void);


/* ---------- Event flags ---------- */

#define EVT_BIT_RUN            (1u << 0)
#define EVT_BIT_STOP           (1u << 1)
#define EVT_BIT_BALL_DETECTED  (1u << 2)
#define EVT_BIT_CALIBRATED     (1u << 3)
#define EVT_BIT_FAULT          (1u << 4)
#define EVT_BIT_BOOT_DONE      (1u << 5)
#define EVT_BIT_TFT_READY      (1u << 6)

void system_state_init(void);


/* ---------- Fault status ---------- */

/* Return true when the system fault flag is set. */
bool system_state_fault_is_set(void);

#endif

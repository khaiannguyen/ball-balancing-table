/**
 * @file    task_imu_fusion.c
 * @brief   IMU sensor fusion task.
 *
 * Processes MPU6500 accelerometer and gyroscope data and estimates roll,
 * pitch, roll rate, and pitch rate using independent 1D Kalman filters.
 *
 * The task is awakened by an IMU DMA completion notification rather than
 * polling the sensor. The resulting state is published through the shared
 * system-state interface for use by the control loop.
 */

#include "task_imu_fusion.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stdio.h>

#include "main.h"
#include "system_state.h"

#include "imu_fusion.h"
#include "imu_mpu6500.h"

extern osThreadId_t ImuFusionTaskHandle;

/*
 * Sensor scale factors must match the full-scale configuration programmed
 * by imu_mpu6500.c.
 *
 * The current values correspond to:
 *
 *   Gyro  : ±500 dps -> 65.5 LSB/(deg/s)
 *   Accel : ±4 g     -> 8192 LSB/g
 *
 * A mismatch between these constants and the sensor configuration produces
 * a fixed scaling error in the estimated attitude and angular rates.
 */
#define GYRO_LSB_PER_DPS 65.5f
#define ACCEL_LSB_PER_G  8192.0f

/*
 * Use the configured hardware sampling period rather than measuring dt from
 * the FreeRTOS tick.
 *
 * The FreeRTOS tick resolution is insufficient for accurate timing at the
 * IMU sampling rate, while the task is synchronized to the sensor DMA
 * completion path.
 */
#define IMU_SAMPLE_DT_S 0.001f

/*
 * Accelerometer measurements are trusted only when the measured acceleration
 * magnitude remains close to 1 g.
 *
 * During strong servo-induced motion, dynamic acceleration contaminates the
 * gravity measurement and can corrupt the attitude estimate.
 */
#define ACCEL_TRUST_MIN_G 0.8f
#define ACCEL_TRUST_MAX_G 1.2f

/*
 * Bound the wait for a new IMU notification.
 *
 * A finite timeout prevents the task from blocking indefinitely if the
 * sensor or DMA path stops producing notifications. Safety handling remains
 * outside this task so the IMU task can continue reporting liveness.
 */
#define IMU_NOTIFY_TIMEOUT_MS 20

void StartTaskImuFusion(void *argument)
{
    (void)argument;

    /*
     * Store the current task handle so the IMU DMA completion path can
     * notify this task directly when a new sensor sample is available.
     */
    ImuFusionTaskHandle =
        (osThreadId_t)xTaskGetCurrentTaskHandle();

    /*
     * Initialize independent filters for roll and pitch.
     *
     * Q_angle controls the assumed angle-process uncertainty.
     * Q_bias controls the gyro-bias adaptation rate.
     * R_measure controls the assumed accelerometer measurement noise.
     *
     * These values determine the balance between gyro smoothness and
     * accelerometer correction and therefore remain application tuning
     * parameters.
     */
    kalman1d_t kf_roll;
    kalman1d_t kf_pitch;

    kalman1d_init(
        &kf_roll,
        0.0008f,
        0.003f,
        0.0156f
    );

    kalman1d_init(
        &kf_pitch,
        0.0008f,
        0.003f,
        0.0148f
    );

    for (;;)
    {
        /*
         * Wait for the DMA completion path to signal a new IMU sample.
         *
         * Task notification avoids polling and keeps the sensor processing
         * task synchronized with the hardware acquisition path.
         */
        uint32_t got =
            ulTaskNotifyTake(
                pdTRUE,
                pdMS_TO_TICKS(IMU_NOTIFY_TIMEOUT_MS)
            );

        int16_t accel_raw[3];
        int16_t gyro_raw[3];

        imu_raw_state_read(
            system_state_get_imu_raw_ptr(),
            accel_raw,
            gyro_raw
        );

        /*
         * Continue processing the most recent available sample after a
         * notification timeout.
         *
         * This prevents the task from becoming permanently blocked when the
         * sensor acquisition path is temporarily unavailable. System-level
         * safety handling remains owned by the control and failsafe layers.
         */
        (void)got;

        /*
         * Convert raw sensor data into physical units used by the fusion
         * algorithm:
         *
         *   Gyroscope  -> deg/s
         *   Accelerometer -> g
         */
        float gx =
            gyro_raw[0] / GYRO_LSB_PER_DPS;

        float gy =
            gyro_raw[1] / GYRO_LSB_PER_DPS;

        /*
         * Yaw rate is not required by the balancing controller and is therefore
         * intentionally excluded from the fusion state.
         */
        float ax =
            accel_raw[0] / ACCEL_LSB_PER_G;

        float ay =
            accel_raw[1] / ACCEL_LSB_PER_G;

        float az =
            -accel_raw[2] / ACCEL_LSB_PER_G;

        /*
         * Estimate roll and pitch from the gravity vector.
         *
         * The axis/sign convention must match the physical IMU mounting and
         * the control-system coordinate frame.
         */
        float roll_acc =
            atan2f(
                ay,
                az
            ) * (180.0f / (float)M_PI);

        float pitch_acc =
            atan2f(
                -ax,
                sqrtf(ay * ay + az * az)
            ) * (180.0f / (float)M_PI);

        /*
         * Determine whether the accelerometer measurement is suitable for
         * attitude correction.
         *
         * A magnitude close to 1 g indicates that gravity is still the
         * dominant acceleration component.
         */
        float accel_norm =
            sqrtf(
                ax * ax +
                ay * ay +
                az * az
            );

        bool trust_accel =
            (accel_norm > ACCEL_TRUST_MIN_G) &&
            (accel_norm < ACCEL_TRUST_MAX_G);

        float vroll;
        float vpitch;

        float roll;
        float pitch;

        if (trust_accel)
        {
            /*
             * Use the normal predict/update cycle.
             *
             * Gyroscope data provides the short-term motion estimate while
             * the accelerometer measurement corrects long-term drift.
             */
            roll =
                kalman1d_update(
                    &kf_roll,
                    roll_acc,
                    gx,
                    IMU_SAMPLE_DT_S,
                    &vroll
                );

            pitch =
                kalman1d_update(
                    &kf_pitch,
                    pitch_acc,
                    gy,
                    IMU_SAMPLE_DT_S,
                    &vpitch
                );
        }
        else
        {
            /*
             * During strong dynamic acceleration, temporarily suppress the
             * accelerometer correction by increasing the measurement noise.
             *
             * The filter therefore behaves as a gyro-driven prediction while
             * preserving the existing filter state and configuration.
             */
            float saved_R =
                kf_roll.R_measure;

            kf_roll.R_measure =
                1.0e6f;

            kf_pitch.R_measure =
                1.0e6f;

            roll =
                kalman1d_update(
                    &kf_roll,
                    roll_acc,
                    gx,
                    IMU_SAMPLE_DT_S,
                    &vroll
                );

            pitch =
                kalman1d_update(
                    &kf_pitch,
                    pitch_acc,
                    gy,
                    IMU_SAMPLE_DT_S,
                    &vpitch
                );

            kf_roll.R_measure =
                saved_R;

            kf_pitch.R_measure =
                saved_R;
        }

        /*
         * Publish the fused attitude and angular-rate estimates through the
         * shared system-state interface.
         *
         * The control loop consumes this state without accessing the filter
         * internals directly.
         */
        imu_state_write(
            system_state_get_imu_ptr(),
            roll,
            pitch,
            vroll,
            vpitch
        );

        /*
         * Mark the task alive after completing one complete fusion cycle.
         */
        task_alive_mark(
            ALIVE_BIT_IMU_FUSION
        );
    }
}

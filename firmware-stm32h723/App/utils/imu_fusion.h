#ifndef IMU_FUSION_H
#define IMU_FUSION_H

/*
 * Two-state Kalman filter [angle, gyro_bias] for one axis.
 *
 * Two independent instances are used for roll and pitch.
 *
 * Model:
 *   angle_dot = gyro_rate - bias
 *   bias_dot  = 0
 *
 * Measurement:
 *   angle_accel from accelerometer.
 */
typedef struct {
    float angle;      // Estimated angle (deg)
    float bias;       // Estimated gyro bias (deg/s)
    float P[2][2];    // Error covariance matrix

    float Q_angle;    // Process noise for gyro integration
    float Q_bias;     // Process noise for gyro bias drift
    float R_measure;  // Measurement noise for accelerometer
} kalman1d_t;

void kalman1d_init(
    kalman1d_t *kf,
    float Q_angle,
    float Q_bias,
    float R_measure
);

/* Update the filter using gyro rate and accelerometer angle.
 *
 * newRate is the raw gyro rate in deg/s.
 * newAngle is the accelerometer angle in degrees.
 * Returns the filtered angle.
 *
 * out_rate_unbiased receives the bias-corrected gyro rate.
 */
float kalman1d_update(
    kalman1d_t *kf,
    float newAngle,
    float newRate,
    float dt,
    float *out_rate_unbiased
);

#endif

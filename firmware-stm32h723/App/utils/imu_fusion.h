#ifndef IMU_FUSION_H
#define IMU_FUSION_H

/*
 * Kalman Filter 2 trạng thái [angle, gyro_bias] cho 1 trục — thuật toán kinh
 * điển dùng trong robot tự cân bằng (tương tự Kalman.h của Kristian Lauszus,
 * đã kiểm chứng rộng rãi). Dùng 2 instance độc lập cho roll và pitch, KHÔNG
 * dùng chung 1 EKF 6 trạng thái với yaw — vì bàn không cần yaw, và tách trục
 * giúp code đơn giản, nhẹ, dễ tune hơn nhiều so với EKF tổng quát.
 *
 * Model:
 *   angle_dot = gyro_rate - bias        (input: gyro đã trừ bias)
 *   bias_dot  = 0 (random walk chậm)
 *   Đo lường:  angle_accel (tính từ accelerometer qua atan2)
 */

typedef struct {
    float angle;      // góc ước lượng hiện tại (deg)
    float bias;       // bias gyro ước lượng hiện tại (deg/s)
    float P[2][2];     // ma trận hiệp phương sai sai số

    float Q_angle;    // process noise - độ tin cậy vào tích phân gyro
    float Q_bias;      // process noise - tốc độ trôi của bias
    float R_measure;   // measurement noise - độ tin cậy vào accel
} kalman1d_t;

void  kalman1d_init(kalman1d_t *kf, float Q_angle, float Q_bias, float R_measure);

/* Gọi mỗi chu kỳ (dt giây): newRate = gyro thô (deg/s) TRỰC TIẾP từ cảm biến
 * (chưa trừ bias - hàm tự trừ bias nội bộ), newAngle = góc tính từ accel (deg).
 * Trả về góc đã lọc (deg). Đọc thêm bias đã lọc qua kf->bias, rate đã bù bias
 * qua tham số ra out_rate_unbiased (dùng làm vroll/vpitch). */
float kalman1d_update(kalman1d_t *kf, float newAngle, float newRate, float dt,
                       float *out_rate_unbiased);

#endif

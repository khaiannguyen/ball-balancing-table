#include "imu_fusion.h"
#include <string.h>

void kalman1d_init(kalman1d_t *kf, float Q_angle, float Q_bias, float R_measure)
{
    memset(kf, 0, sizeof(*kf));
    kf->Q_angle   = Q_angle;
    kf->Q_bias     = Q_bias;
    kf->R_measure  = R_measure;
    /* P khởi tạo = 0 là chấp nhận được (không phải identity) vì ta CHƯA biết
     * gì về sai số ban đầu - Kalman sẽ tự "học" nhanh trong vài chục ms đầu
     * nhờ measurement update liên tục ở 1kHz. */
}

float kalman1d_update(kalman1d_t *kf, float newAngle, float newRate, float dt,
                       float *out_rate_unbiased)
{
    /* ---- Predict ---- */
    float rate = newRate - kf->bias;
    kf->angle += dt * rate;

    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* ---- Update (measurement từ accel) ---- */
    float S = kf->P[0][0] + kf->R_measure;    /* innovation covariance, luôn > 0 */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    float y = newAngle - kf->angle;           /* innovation (residual) */
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K0 * P00_temp;
    kf->P[0][1] -= K0 * P01_temp;
    kf->P[1][0] -= K1 * P00_temp;
    kf->P[1][1] -= K1 * P01_temp;

    if (out_rate_unbiased) {
        *out_rate_unbiased = newRate - kf->bias;   /* dùng làm vroll/vpitch */
    }
    return kf->angle;
}

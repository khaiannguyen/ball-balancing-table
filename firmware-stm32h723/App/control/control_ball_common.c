#include "control_ball_common.h"
#include "calibration_data.h"
#include "servo_actuator.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>

/* SỬA (Giai đoạn 6) - IK giờ là đa thức BẬC 2 trực tiếp từ ik_coef[3][6]
 * (calibration_data v3), thay cho công thức bậc 1 tạm gán qua A1..B3 của
 * bản trước - xem giải thích đầy đủ trong control_ball_common.h. S3 đọc
 * thẳng từ ik_coef[2][*] (đã được tính sẵn lúc calib để luôn thỏa
 * S3=-(S1+S2) với MỌI roll/pitch), KHÔNG suy runtime bằng phép trừ nữa. */
void control_ball_ik(float roll_d, float pitch_d, float *s1, float *s2, float *s3)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    const float R = roll_d;
    const float P = pitch_d;
    const float R2 = R * R;
    const float P2 = P * P;
    const float RP = R * P;

    /* f[6] = [R, P, R^2, P^2, R*P, 1] - PHẢI khớp đúng thứ tự feature_row()
     * dùng lúc giải Least Squares trong control_mode_calib.c, nếu đổi thứ
     * tự 1 bên mà không đổi bên kia thì hệ số sẽ bị gán sai vị trí. */
    const float f[6] = { R, P, R2, P2, RP, 1.0f };

    float out[3];
    for (int i = 0; i < 3; i++) {
        float acc = 0.0f;
        for (int k = 0; k < 6; k++) {
            acc += c->ik_coef[i][k] * f[k];
        }
        out[i] = acc;
    }

    *s1 = out[0];
    *s2 = out[1];
    *s3 = out[2];
}

/* ---- THEM (Ke hoach 2): clamp mm va clamp us dung chung cho ca
 * height_offset_us() lan apply_rph(). Static noi bo file, khong can khai
 * bao trong header. ---- */
static float clamp_f(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float control_ball_height_offset_us(float height_d_mm)
{
    /* Clamp vao vung tuyen tinh da xac nhan thuc te - AN TOAN, tranh
     * ngoai suy sai ngoai khoang da do (xem HEIGHT_D_MIN_MM/MAX_MM trong
     * header). */
    height_d_mm = clamp_f(height_d_mm, HEIGHT_D_MIN_MM, HEIGHT_D_MAX_MM);

    /* He so DOI XUNG (ban dieu chinh moi nhat tu nguoi dung): 290us <->
     * 13.0mm CA 2 CHIEU (nang/ha nhu nhau) - CHI 1 he so duy nhat, khac
     * ban nhap dau tien trong plan (18.125/17.935 us/mm lech nhau). */
    static const float K_US_PER_MM = 290.0f / 13.0f; /* ~= 22.3077 us/mm */

    /* Dau AM: height_d duong (nang ban) -> offset us AM (giam xung). */
    return -height_d_mm * K_US_PER_MM;
}

void control_ball_apply_rp(float roll_d, float pitch_d)
{
    /* THEM (Ke hoach 2): gio la wrapper cua apply_rph voi height=0, hanh
     * vi giu NGUYEN 100% so voi ban cu (moi noi dang goi ham nay, vd
     * control_mode_position.c, KHONG can sua gi). */
    control_ball_apply_rph(roll_d, pitch_d, 0.0f);
}

void control_ball_apply_rph(float roll_d, float pitch_d, float height_d)
{
    const calibration_data_t *c = calibration_data_get_ptr();
    float s1, s2, s3;
    control_ball_ik(roll_d, pitch_d, &s1, &s2, &s3);
    //printf("%.2f,%.2f,%.2f,roll_d = %.2f,pitch_d = %.2f\r\n", s1, s2, s3, roll_d, pitch_d);

    /* THEM (Ke hoach 2): cong DEU offset chieu cao vao ca 3 truc, roi
     * clamp TONG (khong phai clamp rieng tung thanh phan roi moi cong -
     * dung dung yeu cau an toan da chot trong plan muc 2.2: "clamp tong,
     * khong chi clamp tung thanh phan rieng le"). */
    const float h_off = control_ball_height_offset_us(height_d);
    s1 = clamp_f(s1 + h_off, -AXIS_OFFSET_MAX_US, AXIS_OFFSET_MAX_US);
    s2 = clamp_f(s2 + h_off, -AXIS_OFFSET_MAX_US, AXIS_OFFSET_MAX_US);
    s3 = clamp_f(s3 + h_off, -AXIS_OFFSET_MAX_US, AXIS_OFFSET_MAX_US);

    servo_actuator_set_target(SERVO_CH_S1, c->S1_neutral + (int32_t)lroundf(s1));
    servo_actuator_set_target(SERVO_CH_S2, c->S2_neutral + (int32_t)lroundf(s2));
    servo_actuator_set_target(SERVO_CH_S3, c->S3_neutral + (int32_t)lroundf(s3));
}

#include "control_ball_common.h"
#include "calibration_data.h"
#include "servo_actuator.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>

void control_ball_ik(
    float roll_d,
    float pitch_d,
    float *s1,
    float *s2,
    float *s3
)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    const float R = roll_d;
    const float P = pitch_d;

    const float R2 = R * R;
    const float P2 = P * P;
    const float RP = R * P;

    /*
     * The feature order is part of the calibration model contract.
     * It must match the order used by control_mode_calib.c when fitting
     * the least-squares coefficients.
     */
    const float f[6] = {
        R,
        P,
        R2,
        P2,
        RP,
        1.0f
    };

    float out[3];

    for (int i = 0; i < 3; i++)
    {
        float acc = 0.0f;

        for (int k = 0; k < 6; k++)
        {
            acc += c->ik_coef[i][k] * f[k];
        }

        out[i] = acc;
    }

    *s1 = out[0];
    *s2 = out[1];
    *s3 = out[2];
}

/* Clamp a value to the specified inclusive range. */
static float clamp_f(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }

    if (v > hi)
    {
        return hi;
    }

    return v;
}

float control_ball_height_offset_us(float height_d_mm)
{
    /*
     * Limit the input to the experimentally validated linear region.
     * This prevents unsupported extrapolation beyond the measured range.
     */
    height_d_mm = clamp_f(
        height_d_mm,
        HEIGHT_D_MIN_MM,
        HEIGHT_D_MAX_MM
    );

    /*
     * The measured response is symmetric:
     *
     *   13.0 mm height change <-> 290 us servo offset
     *
     * A positive height demand raises the platform and therefore requires
     * a negative servo offset.
     */
    static const float K_US_PER_MM = 290.0f / 13.0f;

    return -height_d_mm * K_US_PER_MM;
}

void control_ball_apply_rp(float roll_d, float pitch_d)
{
    /*
     * Keep the roll/pitch-only API as a zero-height wrapper so existing
     * control modes use the same actuator path as height-aware control.
     */
    control_ball_apply_rph(
        roll_d,
        pitch_d,
        0.0f
    );
}

void control_ball_apply_rph(
    float roll_d,
    float pitch_d,
    float height_d
)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    float s1;
    float s2;
    float s3;

    control_ball_ik(
        roll_d,
        pitch_d,
        &s1,
        &s2,
        &s3
    );

    /*
     * Height contributes equally to all three servo offsets.
     *
     * Clamp the combined roll/pitch/height command rather than each
     * component independently so the final actuator command remains
     * within the validated mechanical limit.
     */
    const float h_off = control_ball_height_offset_us(height_d);

    s1 = clamp_f(
        s1 + h_off,
        -AXIS_OFFSET_MAX_US,
        AXIS_OFFSET_MAX_US
    );

    s2 = clamp_f(
        s2 + h_off,
        -AXIS_OFFSET_MAX_US,
        AXIS_OFFSET_MAX_US
    );

    s3 = clamp_f(
        s3 + h_off,
        -AXIS_OFFSET_MAX_US,
        AXIS_OFFSET_MAX_US
    );

    /*
     * IK outputs are offsets from neutral. Convert them to absolute servo
     * targets only at the actuator interface.
     */
    servo_actuator_set_target(
        SERVO_CH_S1,
        c->S1_neutral + (int32_t)lroundf(s1)
    );

    servo_actuator_set_target(
        SERVO_CH_S2,
        c->S2_neutral + (int32_t)lroundf(s2)
    );

    servo_actuator_set_target(
        SERVO_CH_S3,
        c->S3_neutral + (int32_t)lroundf(s3)
    );
}

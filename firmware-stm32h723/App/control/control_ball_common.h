#ifndef CONTROL_BALL_COMMON_H
#define CONTROL_BALL_COMMON_H

#include <stdbool.h>

/*
 * Common ball-control functions shared by Balance and Position modes.
 *
 * The inverse kinematics layer converts roll/pitch control demands into
 * servo offsets using the calibrated second-order model:
 *
 *   S_i = c0*R + c1*P + c2*R^2
 *       + c3*P^2 + c4*R*P + c5
 *
 * The model is calibrated from a 2D sweep and stored in calibration_data_t.
 *
 * roll_d and pitch_d are control deviations around zero. They must not be
 * replaced with absolute platform angles.
 *
 * The IK output is a servo offset in microseconds. Neutral PWM values are
 * added only when the command is applied to the actuator layer.
 *
 * Height control is independent of the IK model. Its offset is applied
 * equally to all three servos after the roll/pitch contribution is computed.
 */

/*
 * Calculate servo offsets from roll and pitch control demands.
 *
 * The function only evaluates the calibrated polynomial and does not
 * modify actuator state.
 */
void control_ball_ik(
    float roll_d,
    float pitch_d,
    float *s1,
    float *s2,
    float *s3
);

/*
 * Calculate roll/pitch IK and apply the resulting servo targets.
 *
 * The function adds each calibrated neutral position before sending the
 * final command to the actuator layer.
 *
 * This is equivalent to control_ball_apply_rph(roll_d, pitch_d, 0.0f).
 */
void control_ball_apply_rp(float roll_d, float pitch_d);

/*
 * Height control range validated by the current calibration data.
 *
 * Values outside this range are clamped before converting to servo offset
 * to avoid extrapolating beyond the measured linear region.
 */
#define HEIGHT_D_MIN_MM (-13.0f)
#define HEIGHT_D_MAX_MM (13.0f)

/*
 * Maximum total servo offset allowed after combining roll, pitch, and height.
 *
 * The clamp is applied after all contributions are combined so that the
 * final actuator command cannot exceed the validated mechanical range.
 */
#define AXIS_OFFSET_MAX_US (360.0f)

/*
 * Convert height demand in millimeters into a common servo offset.
 *
 * Positive height demand raises the platform and therefore produces a
 * negative PWM offset.
 */
float control_ball_height_offset_us(float height_d_mm);

/*
 * Calculate roll/pitch IK, add the height contribution to all three servos,
 * clamp the combined offset, and apply the final actuator targets.
 */
void control_ball_apply_rph(
    float roll_d,
    float pitch_d,
    float height_d
);

#endif /* CONTROL_BALL_COMMON_H */

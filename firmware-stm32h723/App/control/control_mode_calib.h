#ifndef CONTROL_MODE_CALIB_H
#define CONTROL_MODE_CALIB_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Calibration mode identifies the platform's neutral offsets and fits the
 * second-order inverse-kinematics model used by control_ball_common.c.
 *
 * The calibration sweep covers the Roll/Pitch operating region using a 2D
 * grid. For each measured point, the MCU records:
 *
 *   S1, S2, S3, Roll, Pitch
 *
 * The fitted model is:
 *
 *   S_i = c0*R + c1*P + c2*R^2
 *       + c3*P^2 + c4*R*P + c5
 *
 * The calibration is accepted only when the least-squares solution is valid
 * and the measured fit quality satisfies the configured error limits.
 *
 * No system-state transition is published directly from this module.
 * Completion and failure are reported through StateRequestQueueHandle.
 */

typedef enum
{
    CALIB_SUB_OFFSET = 0,

    /*
     * Sweep the calibrated S1/S2 grid while maintaining the mechanical
     * constraint S3 = -(S1 + S2).
     */
    CALIB_SUB_SWEEP_GRID,

    /* Fit and validate the IK model, then prepare persistent data. */
    CALIB_SUB_SAVE,

    /* Calibration completed successfully. */
    CALIB_SUB_DONE,

    /* Calibration failed or produced an invalid model. */
    CALIB_SUB_ERROR

} calib_sub_state_t;

/*
 * Enter calibration mode and initialize the calibration state machine.
 *
 * Any neutral override from a previous calibration session is cleared.
 */
void control_mode_calib_enter(void);

/*
 * Advance the calibration state machine.
 *
 * dt must represent the elapsed control-loop time. The function performs
 * state transitions, servo movement, measurement collection, model fitting,
 * and persistent-data handling according to the current calibration state.
 */
void control_mode_calib_step(float dt);

/* Return the current calibration sub-state. */
calib_sub_state_t control_mode_calib_get_sub_state(void);

/*
 * Override the servo neutral positions for the next calibration run.
 *
 * The override must be set after control_mode_calib_enter() and before the
 * first control_mode_calib_step() call. The supplied neutral values become
 * the center of both the offset measurement and the 2D sweep.
 *
 * The override is cleared automatically when the calibration session ends.
 */
void control_mode_calib_set_neutral(
    int16_t s1_neutral,
    int16_t s2_neutral,
    int16_t s3_neutral
);

#endif /* CONTROL_MODE_CALIB_H */

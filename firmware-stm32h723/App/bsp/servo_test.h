/**
 * @file    servo_test.h
 * @brief   Servo actuator test and characterization interface.
 *
 * This module provides deterministic test routines for validating the
 * three-servo actuator system on the physical platform.
 *
 * Two test interfaces are provided:
 *
 * 1. Legacy cyclic test:
 *      servo_test_init()
 *      servo_test_step()
 *
 *    This routine executes a predefined sequence covering neutral hold,
 *    absolute positioning, incremental positioning, and trajectory return.
 *
 * 2. Controlled test sessions:
 *      servo_test_start()
 *      servo_test_step_dt()
 *      servo_test_stop()
 *
 *    These modes support manual actuator adjustment and synchronized
 *    servo sweep logging for calibration and characterization.
 *
 * The test layer operates above servo_actuator and therefore does not
 * access PWM hardware directly.
 */

#ifndef SERVO_TEST_H
#define SERVO_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the legacy servo test sequence.
 *
 * Initializes the actuator layer and resets the internal state of the
 * predefined four-phase test sequence.
 *
 * This function should be called once before the first call to
 * servo_test_step().
 */
void servo_test_init(void);

/**
 * @brief Execute one step of the legacy servo test sequence.
 *
 * The legacy test is designed for a fixed 100 Hz execution rate
 * with dt = 10 ms per call.
 *
 * The sequence consists of:
 *
 *     1. HOLD_NEUTRAL
 *        Verify that all three servos remain stable at neutral.
 *
 *     2. ABSOLUTE_SWEEP
 *        Exercise absolute positioning on S1 while the actuator
 *        layer applies its configured slew-rate limiting.
 *
 *     3. INCREMENTAL
 *        Apply incremental commands to S2 and verify boundary
 *        handling and actuator limiting.
 *
 *     4. TRAJECTORY_HOME
 *        Move S3 away from neutral and return it using the
 *        trajectory generator.
 *
 * The sequence repeats continuously.
 */
void servo_test_step(void);

/**
 * @brief Servo test operating modes.
 *
 * SERVO_TEST_MODE_LEGACY_B2:
 *     Selects the predefined legacy test sequence driven by
 *     servo_test_init() and servo_test_step().
 *
 * SERVO_TEST_MODE_MANUAL_STEP:
 *     Allows the selected servo to be adjusted incrementally
 *     through explicit commands.
 *
 * SERVO_TEST_MODE_SWEEP_LOG:
 *     Performs a synchronized multi-servo sweep and records the
 *     resulting servo positions together with roll/pitch measurements.
 */
typedef enum
{
    SERVO_TEST_MODE_LEGACY_B2 = 0,
    SERVO_TEST_MODE_MANUAL_STEP,
    SERVO_TEST_MODE_SWEEP_LOG
} servo_test_mode_t;

/**
 * @brief Start a new controlled servo test session.
 *
 * Resets all state associated with the previous session, initializes
 * the requested test mode, builds the active servo-channel list, and
 * emits the CSV header used by the logging interface.
 *
 * @param mode      Test mode to execute. The legacy mode is normally
 *                  controlled through servo_test_init()/servo_test_step().
 *
 * @param servo_ch  Channel selection:
 *                  0 = test all three servos sequentially,
 *                  1 = S1,
 *                  2 = S2,
 *                  3 = S3.
 */
void servo_test_start(servo_test_mode_t mode, uint8_t servo_ch);

/**
 * @brief Stop the current controlled test session.
 *
 * No new test commands are generated after this call. The actuator
 * remains at its current commanded position.
 */
void servo_test_stop(void);

/**
 * @brief Check whether the current test session has completed.
 *
 * Sweep-based tests report completion after all requested channels
 * have been scanned. Manual-step mode does not complete automatically
 * and remains active until explicitly stopped.
 *
 * @return true if the test session has completed; otherwise false.
 */
bool servo_test_is_done(void);

/**
 * @brief Execute one controlled test-cycle update.
 *
 * This function should be called once per control-loop cycle using
 * the actual elapsed time of that cycle.
 *
 * Unlike the legacy servo_test_step(), this interface does not assume
 * a fixed control-loop frequency.
 *
 * @param dt  Actual control-loop period in seconds.
 */
void servo_test_step_dt(float dt);

/**
 * @brief Apply an incremental command to the selected servo.
 *
 * Intended for manual actuator characterization. The command is
 * forwarded to the actuator layer and the resulting actuator state
 * is logged together with the current IMU roll/pitch measurements.
 *
 * @param delta_us  Incremental servo command in microseconds.
 */
void servo_test_manual_adjust(int16_t delta_us);

/**
 * @brief Select the servo used by manual-step mode.
 *
 * @param servo_ch  Servo number:
 *                  1 = S1,
 *                  2 = S2,
 *                  3 = S3.
 */
void servo_test_manual_select_channel(uint8_t servo_ch);

/**
 * @brief Print the CSV column header used by the test logger.
 *
 * Output format:
 *
 *     t_ms,S1_us,S2_us,S3_us,roll_deg,pitch_deg
 */
void servo_test_log_csv_header(void);

/**
 * @brief Print one CSV measurement row.
 *
 * The row contains the current actuator positions and IMU attitude
 * measurements.
 *
 * @param t_ms  Test elapsed time in milliseconds.
 */
void servo_test_log_csv_row(uint32_t t_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_TEST_H */

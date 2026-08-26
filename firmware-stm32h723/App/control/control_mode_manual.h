#ifndef CONTROL_MODE_MANUAL_H
#define CONTROL_MODE_MANUAL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @file    control_mode_manual.h
 * @brief   Manual servo test mode interface.
 *
 * Manual mode provides direct servo adjustment and sweep logging without
 * automatically changing the system state when an operation completes.
 *
 * The UI controls the selected sub-state and servo channel.
 */
typedef enum {
    MANUAL_SUB_IDLE = 0,
    MANUAL_SUB_MANUAL_STEP,
    MANUAL_SUB_SWEEP_LOG,
    MANUAL_SUB_DONE
} manual_sub_state_t;

/**
 * @brief Initialize Manual mode.
 *
 * Stops any active servo test, resets the sub-state to idle, and updates
 * the UI guide text.
 */
void control_mode_manual_enter(void);

/**
 * @brief Execute one Manual control cycle.
 *
 * Starts the selected servo test when entering a new sub-state and advances
 * the active test using the control-loop period.
 *
 * @param dt Control-loop period in seconds.
 */
void control_mode_manual_step(float dt);

/**
 * @brief Get the current Manual sub-state.
 *
 * @return Current manual sub-state.
 */
manual_sub_state_t control_mode_manual_get_sub_state(void);

/**
 * @brief Select the Manual operation.
 *
 * The currently active servo test is stopped before changing the sub-state.
 *
 * @param sub New Manual sub-state.
 */
void control_mode_manual_select_substate(manual_sub_state_t sub);

/**
 * @brief Adjust the selected servo position.
 *
 * The adjustment is forwarded to the servo test service only while
 * MANUAL_SUB_MANUAL_STEP is active.
 *
 * @param delta_us Servo position increment in microseconds.
 */
void control_mode_manual_adjust(int16_t delta_us);

/**
 * @brief Select the servo channel used by manual adjustment.
 *
 * The channel selection is forwarded to the servo test service only while
 * MANUAL_SUB_MANUAL_STEP is active.
 *
 * @param servo_ch Servo channel to select.
 */
void control_mode_manual_select_channel(uint8_t servo_ch);

#endif /* CONTROL_MODE_MANUAL_H */

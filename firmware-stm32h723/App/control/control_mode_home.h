#ifndef CONTROL_MODE_HOME_H
#define CONTROL_MODE_HOME_H

#include <stdbool.h>

/**
 * @file    control_mode_home.h
 * @brief   Home mode control interface.
 *
 * Home mode moves all three servos to their calibrated neutral positions
 * using synchronized trajectories and confirms the final actuator positions
 * before notifying the state machine.
 *
 * The module does not modify the system state directly. Completion is
 * reported through StateRequestQueueHandle using EVT_HOME_DONE.
 */

/**
 * @brief Initialize a new Home motion.
 *
 * Reads the actual servo positions as the trajectory start point and
 * generates synchronized trajectories toward the calibrated neutral
 * positions.
 */
void control_mode_home_enter(void);

/**
 * @brief Execute one Home control cycle.
 *
 * Advances the synchronized trajectories and verifies the actual actuator
 * positions against the calibrated neutral positions. Completion requires
 * all three servos to remain within HOME_TOLERANCE_US for
 * HOME_SETTLE_CYCLES consecutive control cycles.
 *
 * EVT_HOME_DONE is sent only after the physical actuator positions satisfy
 * the home condition.
 *
 * @param dt Control-loop period in seconds.
 */
void control_mode_home_step(float dt);

/**
 * @brief Check whether Home completion has been reported.
 *
 * @return true after EVT_HOME_DONE has been successfully queued.
 */
bool control_mode_home_is_done(void);

#endif /* CONTROL_MODE_HOME_H */

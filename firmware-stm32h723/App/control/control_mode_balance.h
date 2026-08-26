#ifndef CONTROL_MODE_BALANCE_H
#define CONTROL_MODE_BALANCE_H

#include <stdbool.h>

/*
 * Balance mode receives Roll/Pitch/Height setpoints from the Jetson.
 *
 * Setpoints are passed through the trajectory engine before reaching the
 * inverse-kinematics layer. This limits command velocity and acceleration
 * in Roll/Pitch/Height space while the actuator layer provides the final
 * servo-space slew-rate protection.
 *
 * When Ball is disabled or the Jetson connection becomes invalid, the same
 * trajectory engine is used to return all three axes smoothly to zero.
 *
 * This mode does not run a PID loop on the MCU. The MCU executes the
 * commanded trajectory and applies the calibrated IK mapping.
 */

/*
 * Initialize Balance mode.
 *
 * The mode starts with Ball disabled and clears all trajectory state so
 * commands from a previous mode entry cannot affect the new session.
 */
void control_mode_balance_enter(void);

/*
 * Update the Balance trajectory and apply the current command.
 *
 * dt must represent the elapsed time since the previous update. Invalid or
 * excessively large values are ignored to prevent a single delayed task
 * cycle from producing an unintended trajectory jump.
 */
void control_mode_balance_step(float dt);

/*
 * Enable or disable ball balancing.
 *
 * Disabling the ball does not immediately force the servos to neutral.
 * The active trajectory is replanned toward zero so the platform returns
 * smoothly under the normal velocity and acceleration limits.
 */
void control_mode_balance_set_ball_on(bool on);

/* Return the current Ball ON/OFF state. */
bool control_mode_balance_get_ball_on(void);

#endif /* CONTROL_MODE_BALANCE_H */

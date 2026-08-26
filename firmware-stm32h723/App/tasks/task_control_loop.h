/**
 * @file    task_control_loop.h
 * @brief   Main control-loop task interface.
 *
 * Executes the active control mode at a fixed 100 Hz rate while the system
 * is in STATE_RUN.
 *
 * The task is responsible for control-mode dispatching and actuator
 * trajectory advancement.
 */

#ifndef TASK_CONTROL_LOOP_H
#define TASK_CONTROL_LOOP_H

/**
 * @brief Run the main control-loop task.
 *
 * Initializes the IMU, buttons, calibration data, and actuator layer during
 * boot, then dispatches the active control mode at 100 Hz.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskControlLoop(void *argument);

#endif /* TASK_CONTROL_LOOP_H */

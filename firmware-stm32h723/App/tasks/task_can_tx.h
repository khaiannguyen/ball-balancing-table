/**
 * @file    task_can_tx.h
 * @brief   CAN transmission task interface.
 *
 * Provides the FreeRTOS entry point for the periodic CAN status and telemetry
 * publisher.
 */

#ifndef TASK_CAN_TX_H
#define TASK_CAN_TX_H

/**
 * @brief Run the CAN transmission task.
 *
 * Periodically publishes system state, sensor data, actuator feedback,
 * desired setpoints, and the CAN heartbeat.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskCanTx(void *argument);

#endif /* TASK_CAN_TX_H */

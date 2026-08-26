/**
 * @file    task_can_rx.h
 * @brief   CAN receive task interface.
 *
 * Provides the FreeRTOS entry point for processing received CAN messages.
 *
 * The task is responsible for consuming CAN RX data and forwarding decoded
 * messages to the appropriate application-level interfaces.
 */

#ifndef TASK_CAN_RX_H
#define TASK_CAN_RX_H

/**
 * @brief Run the CAN receive task.
 *
 * Processes received CAN frames and updates the corresponding application
 * state through the CAN RX processing path.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskCanRx(void *argument);

#endif /* TASK_CAN_RX_H */

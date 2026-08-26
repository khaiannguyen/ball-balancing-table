/**
 * @file    task_can_bringup_test.h
 * @brief   CAN bring-up test task interface.
 *
 * Declares the FreeRTOS entry point for the deterministic CAN traffic
 * generator used during CAN integration testing.
 */

#ifndef TASK_CAN_BRINGUP_TEST_H
#define TASK_CAN_BRINGUP_TEST_H

/**
 * @brief Run the CAN bring-up test task.
 *
 * Periodically generates deterministic CAN frames for validating the
 * CAN receive path, message decoding, and heartbeat failsafe.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskCanBringupTest(void *argument);

#endif /* TASK_CAN_BRINGUP_TEST_H */

/**
 * @file    task_watchdog.h
 * @brief   System watchdog task interface.
 *
 * Provides the FreeRTOS entry point for supervising the liveness of
 * safety-critical application tasks.
 */

#ifndef TASK_WATCHDOG_H
#define TASK_WATCHDOG_H

/**
 * @brief Run the watchdog supervision task.
 *
 * Monitors the expected task-alive mask and refreshes the independent
 * watchdog only when all required tasks have reported alive.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskWatchdog(void *argument);

#endif /* TASK_WATCHDOG_H */

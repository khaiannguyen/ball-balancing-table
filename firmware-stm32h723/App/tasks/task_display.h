/**
 * @file    task_display.h
 * @brief   Display task interface.
 *
 * Provides the FreeRTOS entry point for TFT initialization, screen
 * management, and periodic UI updates.
 */

#ifndef TASK_DISPLAY_H
#define TASK_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run the display task.
 *
 * Initializes the TFT and screen manager, handles boot synchronization,
 * and updates the active screen at 25 Hz.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskDisplay(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DISPLAY_H */

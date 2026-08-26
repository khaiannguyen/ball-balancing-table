#ifndef TASK_BUTTON_UI_H
#define TASK_BUTTON_UI_H

/**
 * @brief FreeRTOS entry point for the Button UI task.
 *
 * Waits for button events, translates them into logical UI actions, and
 * forwards normal navigation events to the screen manager.
 *
 * BTN1 long press is handled separately because it can request system-state
 * transitions such as RUN, STOP, and SAFE_MODE acknowledgement.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskButtonUi(void *argument);

#endif /* TASK_BUTTON_UI_H */

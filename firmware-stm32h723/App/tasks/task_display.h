#ifndef TASK_DISPLAY_H
#define TASK_DISPLAY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS Display Task
 *
 * Khởi tạo TFT, UI Data, Screen Manager và cập nhật giao diện
 * định kỳ ở 25 Hz.
 */
void StartTaskDisplay(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_DISPLAY_H */

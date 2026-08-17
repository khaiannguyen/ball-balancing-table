#ifndef TASK_CAN_RX_H
#define TASK_CAN_RX_H

/* Entry point cho Task_CAN_RX — gán vào StartCanRx trong freertos.c (CubeMX),
 * hoặc gọi trực tiếp nếu bạn tự tạo task bằng osThreadNew. */
void StartTaskCanRx(void *argument);

#endif

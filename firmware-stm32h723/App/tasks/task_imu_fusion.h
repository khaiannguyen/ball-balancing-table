#ifndef TASK_IMU_FUSION_H
#define TASK_IMU_FUSION_H

#include "cmsis_os2.h"

/* Entry point cho Task_IMU_Fusion — gán vào StartImuFusion trong freertos.c.
 * Task được đánh thức bằng notify từ HAL_SPI_TxRxCpltCallback (imu_mpu6500.c,
 * ISR - dùng TxRxCpltCallback vì read raw dùng HAL_SPI_TransmitReceive_DMA
 * full-duplex, KHÔNG phải HAL_SPI_RxCpltCallback) mỗi khi DMA đọc xong 1 lần
 * accel+gyro mới từ MPU6500 — KHÔNG polling. */
void StartTaskImuFusion(void *argument);

/* Handle của task này — imu_mpu6500.c đã extern osThreadId_t ImuFusionTaskHandle
 * và tự cast sang TaskHandle_t khi gọi vTaskNotifyGiveFromISR - dùng đúng
 * osThreadId_t ở đây để khớp kiểu, tránh lệch typedef giữa 2 file. Định nghĩa
 * thật nằm trong task_imu_fusion.c, gán giá trị ngay đầu StartTaskImuFusion(). */
extern osThreadId_t ImuFusionTaskHandle;

#endif

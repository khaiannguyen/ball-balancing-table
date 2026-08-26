/**
 * @file    task_imu_fusion.h
 * @brief   IMU sensor fusion task interface.
 *
 * Provides the FreeRTOS entry point and task handle used by the IMU
 * acquisition path to notify the fusion task when a new DMA sample is ready.
 */

#ifndef TASK_IMU_FUSION_H
#define TASK_IMU_FUSION_H

#include "cmsis_os2.h"

/**
 * @brief Run the IMU sensor fusion task.
 *
 * Waits for IMU DMA notifications, processes the latest accelerometer and
 * gyroscope samples, and publishes the fused attitude state.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartTaskImuFusion(void *argument);

/**
 * @brief Handle of the IMU fusion task.
 *
 * The IMU acquisition path uses this handle to notify the fusion task when
 * a new accelerometer and gyroscope sample has been received through DMA.
 */
extern osThreadId_t ImuFusionTaskHandle;

#endif /* TASK_IMU_FUSION_H */

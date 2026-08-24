#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * MPU-6500 register definitions.
 *
 * The device is accessed through SPI. Register read transactions
 * require the MSB of the register address to be set, while register
 * write transactions require the MSB to be cleared.
 */

#define MPU6500_WHO_AM_I_REG       0x75
#define MPU6500_WHO_AM_I_VAL       0x70

#define MPU6500_PWR_MGMT_1         0x6B

#define MPU6500_ACCEL_XOUT_H       0x3B
#define MPU6500_GYRO_XOUT_H        0x43

/*
 * Sample-rate divider register.
 *
 * When the DLPF is enabled, the MPU-6500 internal sample rate is
 * divided according to:
 *
 *     Sample Rate = Internal Sample Rate / (1 + SMPLRT_DIV)
 */
#define MPU6500_SMPLRT_DIV         0x19

/*
 * Digital Low-Pass Filter configuration.
 *
 * DLPF_CFG is located in CONFIG[2:0].
 */
#define MPU6500_CONFIG             0x1A
#define MPU6500_GYRO_CONFIG        0x1B
#define MPU6500_ACCEL_CONFIG       0x1C

#define MPU6500_INT_PIN_CFG        0x37
#define MPU6500_INT_ENABLE         0x38
#define MPU6500_INT_STATUS         0x3A

/*
 * PWR_MGMT_1[7]:
 * Hardware reset command.
 */
#define MPU6500_H_RESET_BIT        0x80

/*
 * INT_ENABLE[0]:
 * Enables the Raw Data Ready interrupt.
 *
 * This interrupt is used as the sampling trigger for the STM32
 * acquisition path.
 */
#define MPU6500_INT_ENABLE_RAW_RDY 0x01

/*
 * DLPF configuration used by this application.
 *
 * Enabling the DLPF establishes the intended internal sampling
 * behavior and allows SMPLRT_DIV to control the output data rate.
 *
 * The selected bandwidth is appropriate for the platform
 * stabilization control loop.
 */
#define MPU6500_DLPF_CFG_1KHZ      0x03

/*
 * @brief Initialize the MPU-6500 and configure its SPI interface.
 *
 * The initialization sequence performs a software reset, wakes the
 * device, verifies WHO_AM_I, configures the interrupt source, sensor
 * full-scale ranges, DLPF, and sample-rate divider.
 *
 * The IMU EXTI interrupt is disabled during initialization because
 * initialization uses blocking SPI transactions on the same SPI
 * peripheral used by the DMA acquisition path.
 *
 * @param hspi SPI peripheral handle connected to the MPU-6500.
 *
 * @return true if all required register writes are verified
 *         successfully; otherwise false.
 */
bool imu_mpu6500_init(SPI_HandleTypeDef *hspi);

/*
 * @brief Verify the MPU-6500 device identity.
 *
 * Reads WHO_AM_I and compares it with the expected MPU-6500 value.
 *
 * @return true when the expected device ID is detected.
 */
bool imu_mpu6500_who_am_i_check(void);

/*
 * @brief Read raw accelerometer and gyroscope samples.
 *
 * This function performs a blocking SPI transaction and is intended
 * for initialization, diagnostics, or low-rate polling.
 *
 * The DMA acquisition path should be used for normal real-time
 * sampling.
 *
 * @param accel Output array containing raw X/Y/Z accelerometer data.
 * @param gyro  Output array containing raw X/Y/Z gyroscope data.
 */
void imu_mpu6500_read_raw(int16_t *accel, int16_t *gyro);

/*
 * @brief Start one DMA acquisition of accelerometer and gyroscope data.
 *
 * The function is normally triggered by the MPU-6500 Data Ready EXTI.
 * It returns immediately after starting the SPI DMA transaction.
 *
 * If the SPI peripheral is still busy with the previous transaction,
 * the new acquisition is discarded rather than overlapping two DMA
 * transfers on the same SPI peripheral.
 */
void imu_read_dma_start(void);

/*
 * @brief Return the number of completed DMA samples and reset the counter.
 *
 * This counter is intended for acquisition-rate monitoring and
 * diagnostics.
 *
 * @return Number of successfully completed DMA acquisitions since
 *         the previous call.
 */
uint32_t imu_get_and_reset_sample_count(void);

/*
 * @brief Print SPI/DMA acquisition diagnostics.
 *
 * Reports DMA start attempts, transfers skipped because SPI was busy,
 * DMA start failures, SPI error callbacks, and the most recent
 * HAL SPI error code.
 */
void imu_debug_print_dma_counters(void);

/*
 * @brief Print selected MPU-6500 registers for diagnostics.
 *
 * The IMU EXTI interrupt is temporarily disabled while the register
 * dump is performed to prevent a new DMA acquisition from starting
 * concurrently with the blocking SPI transactions.
 */
void imu_debug_dump_regs(void);

#endif /* IMU_MPU6500_H */

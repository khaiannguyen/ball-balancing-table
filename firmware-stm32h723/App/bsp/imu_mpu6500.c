#include "imu_mpu6500.h"
#include <stdio.h>
#include "main.h"

#include "system_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

static SPI_HandleTypeDef *s_hspi;

/*
 * SPI DMA transaction length:
 *
 *     1 byte  - register address
 *     6 bytes - accelerometer X/Y/Z
 *     2 bytes - temperature
 *     6 bytes - gyroscope X/Y/Z
 *
 * Total: 15 bytes.
 */
#define IMU_DMA_BUF_LEN               15

/*
 * Cortex-M7 D-Cache line size.
 *
 * The DMA buffers are aligned and sized to a full cache-line boundary
 * so that cache maintenance operations can safely cover the entire
 * DMA memory region.
 */
#define IMU_DMA_BUF_LEN_ALIGNED       32

__attribute__((aligned(32)))
static uint8_t s_dma_tx_buf[IMU_DMA_BUF_LEN_ALIGNED];

__attribute__((aligned(32)))
static uint8_t s_dma_rx_buf[IMU_DMA_BUF_LEN_ALIGNED];

static inline void imu_cs_low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void imu_cs_high(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static volatile uint32_t s_imu_sample_count = 0;

extern osThreadId_t ImuFusionTaskHandle;

/*
 * DMA acquisition diagnostics.
 *
 * These counters distinguish between:
 *
 *     - interrupt/acquisition requests,
 *     - requests rejected because SPI is busy,
 *     - DMA start failures,
 *     - SPI error callbacks.
 *
 * They are useful for verifying the health and timing of the
 * real-time acquisition path.
 */
static volatile uint32_t s_dma_start_attempts = 0;
static volatile uint32_t s_dma_start_busy_skip = 0;
static volatile uint32_t s_dma_start_fail = 0;
static volatile uint32_t s_dma_error_count = 0;
static volatile uint32_t s_last_spi_error_code = 0;

/*
 * @brief Start an SPI DMA acquisition from the MPU-6500.
 *
 * This function is normally called from the IMU Data Ready EXTI path.
 *
 * The SPI state check is intentionally performed inside this function
 * rather than in the caller. This keeps the acquisition API safe for
 * any caller and prevents overlapping DMA transactions on the same
 * SPI peripheral.
 *
 * If the previous transaction is still active, the new sample request
 * is discarded. Waiting inside the interrupt handler would introduce
 * unnecessary interrupt latency and could compromise real-time
 * behavior.
 */
void imu_read_dma_start(void)
{
    s_dma_start_attempts++;

    /*
     * Only start a new transaction when SPI2 is idle.
     *
     * The IMU Data Ready interrupt can arrive again before the previous
     * DMA transfer has completed. Overlapping transfers are not allowed
     * because both transactions use the same SPI peripheral and chip
     * select signal.
     */
    if (HAL_SPI_GetState(s_hspi) != HAL_SPI_STATE_READY)
    {
        s_dma_start_busy_skip++;
        return;
    }

    /*
     * Set the read command for ACCEL_XOUT_H.
     *
     * Setting the MSB selects a register read operation.
     * The remaining transmit bytes are dummy bytes required to clock
     * the corresponding receive data from the MPU-6500.
     */
    s_dma_tx_buf[0] = MPU6500_ACCEL_XOUT_H | 0x80;

    /*
     * The transmit buffer is read by DMA directly from RAM.
     * Clean the D-Cache before starting the transfer so that the DMA
     * engine observes the latest CPU-written buffer contents.
     */
    SCB_CleanDCache_by_Addr((uint32_t *)s_dma_tx_buf, IMU_DMA_BUF_LEN_ALIGNED);

    /*
     * The remaining transmit bytes are fixed dummy values.
     * They are initialized once during system startup and therefore
     * do not need to be cleared for every acquisition.
     */
    imu_cs_low();

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive_DMA(s_hspi, s_dma_tx_buf, s_dma_rx_buf, IMU_DMA_BUF_LEN);

    if (st != HAL_OK)
    {
        /*
         * DMA did not start, so the completion callback will not
         * release chip select. Release it here to prevent the IMU
         * from remaining selected after a failed transaction.
         */
        s_dma_start_fail++;

        imu_cs_high();

        printf("DMA start FAIL: st=%d hal_error=%lu\r\n", st, HAL_SPI_GetError(s_hspi));
    }
}

/*
 * @brief Handle SPI errors associated with the IMU DMA transaction.
 *
 * HAL_SPI_TransmitReceive_DMA() uses the full-duplex transfer path,
 * so an internal SPI error can terminate the transaction without
 * reaching the normal transfer-complete callback.
 *
 * Releasing CS here guarantees that the MPU-6500 SPI transaction is
 * terminated cleanly even when the peripheral reports an error.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_dma_error_count++;
        s_last_spi_error_code = HAL_SPI_GetError(hspi);

        /*
         * Always release chip select when an SPI transfer terminates
         * abnormally.
         */
        imu_cs_high();
    }
}

/*
 * @brief Print DMA acquisition diagnostics.
 *
 * This function is intended for development-time diagnostics and
 * does not participate in the real-time acquisition path.
 */
void imu_debug_print_dma_counters(void)
{
    printf("DMA attempts=%lu busy_skip=%lu fail=%lu spi_error=%lu last_err_code=0x%08lX\r\n", s_dma_start_attempts, s_dma_start_busy_skip, s_dma_start_fail, s_dma_error_count, s_last_spi_error_code);
}

/*
 * @brief Read a single MPU-6500 register using blocking SPI.
 *
 * Register access is used during initialization and diagnostics.
 * The DMA acquisition path is intentionally separate so that
 * configuration transactions cannot interfere with normal sampling.
 */
static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };

    imu_cs_low();

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, HAL_MAX_DELAY);

    imu_cs_high();

    if (st != HAL_OK)
    {
        printf("SPI read err reg 0x%02X: st=%d, hal_error=%lu\r\n", reg, st, HAL_SPI_GetError(s_hspi));
    }

    return rx[1];
}

/*
 * @brief Write a single MPU-6500 register using blocking SPI.
 *
 * @return true when the SPI transaction completes successfully.
 */
static bool imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };
    uint8_t rx[2] = { 0 };

    imu_cs_low();

    HAL_StatusTypeDef st =
        HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, HAL_MAX_DELAY);

    imu_cs_high();

    if (st != HAL_OK)
    {
        printf("SPI write err reg 0x%02X: st=%d, hal_error=%lu\r\n", reg, st, HAL_SPI_GetError(s_hspi));

        return false;
    }

    return true;
}

/*
 * @brief Write a register and verify the value by reading it back.
 *
 * Readback verification is used during initialization to detect
 * communication or configuration failures before enabling the
 * real-time acquisition path.
 */
static bool imu_write_reg_verify(uint8_t reg, uint8_t value)
{
    if (!imu_write_reg(reg, value))
    {
        return false;
    }

    HAL_Delay(1);

    uint8_t rb = imu_read_reg(reg);

    if (rb != value)
    {
        printf("IMU write-verify FAIL: reg 0x%02X wrote 0x%02X, read back 0x%02X\r\n", reg, value, rb);

        return false;
    }

    printf("IMU write-verify OK: reg 0x%02X = 0x%02X\r\n", reg, value);

    return true;
}

/*
 * @brief Initialize the MPU-6500.
 *
 * Initialization uses blocking SPI transactions while normal
 * acquisition uses SPI DMA. Both paths share the same SPI handle,
 * so the IMU Data Ready interrupt must remain disabled throughout
 * initialization.
 *
 * Initialization sequence:
 *
 *     1. Disable IMU EXTI.
 *     2. Ensure chip select is inactive.
 *     3. Reset the MPU-6500.
 *     4. Wake the device.
 *     5. Verify WHO_AM_I.
 *     6. Configure interrupt behavior.
 *     7. Configure gyroscope and accelerometer ranges.
 *     8. Enable the selected DLPF configuration.
 *     9. Configure the sample-rate divider.
 *    10. Clear any pending EXTI request.
 *    11. Re-enable the IMU EXTI interrupt.
 *
 * Disabling EXTI before the first SPI transaction is important because
 * the MPU-6500 may retain its previous interrupt configuration across
 * an MCU-only reset. A stale Data Ready interrupt could otherwise
 * start a DMA transaction while initialization is performing a
 * blocking SPI transaction on the same peripheral.
 */
bool imu_mpu6500_init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;

    /*
     * Prevent the real-time DMA path from starting while the blocking
     * initialization transactions are in progress.
     */
    HAL_NVIC_DisableIRQ(IMU_INT_EXTI_IRQn);

    imu_cs_high();

    /*
     * Allow the SPI/IMU interface to settle before accessing the device.
     */
    HAL_Delay(50);

    /*
     * Perform a hardware-level software reset of the MPU-6500.
     */
    imu_write_reg(MPU6500_PWR_MGMT_1, MPU6500_H_RESET_BIT);

    /*
     * The MPU-6500 requires a delay after reset before register access.
     */
    HAL_Delay(100);

    /*
     * Clear the sleep state and return the device to normal operation.
     */
    imu_write_reg(MPU6500_PWR_MGMT_1, 0x00);

    HAL_Delay(10);

    /*
     * Verify that the expected MPU-6500 is responding before applying
     * the remaining configuration.
     */
    if (!imu_mpu6500_who_am_i_check())
    {
        /*
         * Restore the interrupt configuration before returning so that
         * the rest of the system can continue operating normally.
         */
        HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);

        return false;
    }

    /*
     * Configure the interrupt pin behavior.
     */
    bool ok1 =
        imu_write_reg_verify(MPU6500_INT_PIN_CFG, 0x00);

    /*
     * Enable the Raw Data Ready interrupt.
     *
     * This interrupt provides the sampling trigger used by the
     * STM32 acquisition pipeline.
     */
    bool ok2 =
        imu_write_reg_verify(MPU6500_INT_ENABLE, MPU6500_INT_ENABLE_RAW_RDY);

    /*
     * Configure the gyroscope full-scale range to ±500 dps.
     */
    bool ok3 =
        imu_write_reg_verify(MPU6500_GYRO_CONFIG, 0x08);

    /*
     * Configure the accelerometer full-scale range to ±4 g.
     */
    bool ok4 =
        imu_write_reg_verify(MPU6500_ACCEL_CONFIG, 0x08);

    /*
     * Enable the selected digital low-pass filter configuration.
     *
     * The DLPF configuration establishes the internal sampling
     * behavior used by the selected control-loop data rate.
     */
    bool ok5 =
        imu_write_reg_verify(MPU6500_CONFIG, MPU6500_DLPF_CFG_1KHZ);

    /*
     * SMPLRT_DIV = 0 gives the selected 1 kHz output data rate.
     */
    bool ok6 =
        imu_write_reg_verify(MPU6500_SMPLRT_DIV, 0x00);

    /*
     * Read back the key configuration values for diagnostic output.
     */
    uint8_t who_recheck =
        imu_read_reg(MPU6500_WHO_AM_I_REG);

    printf("INT=%d EN=%d GYRO=%d ACCEL=%d CONFIG=%d SMPLRT=%d WHO=0x%02X\r\n", ok1, ok2, ok3, ok4, ok5, ok6, who_recheck);

    printf("PWR_MGMT_2 (0x6C) = 0x%02X\r\n", imu_read_reg(0x6C));

    /*
     * A pending EXTI request may have been generated while the
     * interrupt was disabled. Clear it before enabling the NVIC
     * interrupt to prevent an immediate stale trigger.
     */
    __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT_Pin);

    HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);

    return ok1 && ok2 && ok3 && ok4 && ok5 && ok6;
}

/*
 * @brief Verify the MPU-6500 device identity.
 */
bool imu_mpu6500_who_am_i_check(void)
{
    uint8_t who =
        imu_read_reg(MPU6500_WHO_AM_I_REG);

    return (who == MPU6500_WHO_AM_I_VAL);
}

/*
 * @brief Read one complete accelerometer/gyroscope sample.
 *
 * This is the blocking polling implementation. The normal runtime
 * acquisition path uses imu_read_dma_start() instead.
 *
 * SPI transaction layout:
 *
 *     Byte 0      : register address
 *     Bytes 1..6  : accelerometer X/Y/Z
 *     Bytes 7..8  : temperature
 *     Bytes 9..14 : gyroscope X/Y/Z
 */
void imu_mpu6500_read_raw(int16_t *accel, int16_t *gyro)
{
    uint8_t tx[15] = { 0 };
    uint8_t rx[15] = { 0 };

    tx[0] = MPU6500_ACCEL_XOUT_H | 0x80;

    imu_cs_low();

    HAL_SPI_TransmitReceive(s_hspi, tx, rx, 15, HAL_MAX_DELAY);

    imu_cs_high();

    accel[0] = (int16_t)((rx[1]  << 8) | rx[2]);
    accel[1] = (int16_t)((rx[3]  << 8) | rx[4]);
    accel[2] = (int16_t)((rx[5]  << 8) | rx[6]);

    gyro[0] = (int16_t)((rx[9]  << 8) | rx[10]);
    gyro[1] = (int16_t)((rx[11] << 8) | rx[12]);
    gyro[2] = (int16_t)((rx[13] << 8) | rx[14]);
}

/*
 * @brief Handle completion of a full-duplex SPI DMA transaction.
 *
 * HAL_SPI_TransmitReceive_DMA() completes through
 * HAL_SPI_TxRxCpltCallback(), not HAL_SPI_RxCpltCallback().
 *
 * The callback:
 *
 *     1. Releases chip select.
 *     2. Invalidates the RX buffer cache.
 *     3. Extracts the accelerometer and gyroscope samples.
 *     4. Publishes the raw sample to the system state.
 *     5. Increments the completed-sample counter.
 *     6. Notifies the IMU fusion task from ISR context.
 *
 * The callback therefore performs only the work required to make
 * the newly acquired sample available to the real-time pipeline.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        imu_cs_high();

        /*
         * DMA writes directly to physical RAM and does not update
         * the Cortex-M7 D-Cache. Invalidate the corresponding cache
         * lines before the CPU reads the received data.
         */
        SCB_InvalidateDCache_by_Addr((uint32_t *)s_dma_rx_buf, IMU_DMA_BUF_LEN_ALIGNED);

        int16_t accel[3];
        int16_t gyro[3];

        accel[0] =
            (int16_t)((s_dma_rx_buf[1] << 8) |
                      s_dma_rx_buf[2]);

        accel[1] =
            (int16_t)((s_dma_rx_buf[3] << 8) |
                      s_dma_rx_buf[4]);

        accel[2] =
            (int16_t)((s_dma_rx_buf[5] << 8) |
                      s_dma_rx_buf[6]);

        gyro[0] =
            (int16_t)((s_dma_rx_buf[9] << 8) |
                      s_dma_rx_buf[10]);

        gyro[1] =
            (int16_t)((s_dma_rx_buf[11] << 8) |
                      s_dma_rx_buf[12]);

        gyro[2] =
            (int16_t)((s_dma_rx_buf[13] << 8) |
                      s_dma_rx_buf[14]);

        /*
         * Publish the sample before notifying the fusion task so that
         * the task always observes a complete and consistent sample.
         */
        imu_raw_state_write(system_state_get_imu_raw_ptr(), accel, gyro);

        s_imu_sample_count++;

        /*
         * Notify the fusion task directly from ISR context.
         *
         * The task can then process the newly acquired sample without
         * polling the DMA state.
         */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveFromISR((TaskHandle_t)ImuFusionTaskHandle, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * The GPIO EXTI callback is intentionally implemented in
 * exti_dispatch.c.
 *
 * Keeping a single EXTI dispatcher prevents multiple definitions of
 * HAL_GPIO_EXTI_Callback() and provides one central routing point for
 * all GPIO interrupt sources.
 */
uint32_t imu_get_and_reset_sample_count(void)
{
    uint32_t count = s_imu_sample_count;

    s_imu_sample_count = 0;

    return count;
}

/*
 * @brief Dump selected MPU-6500 registers.
 *
 * This diagnostic function temporarily disables the IMU EXTI interrupt
 * and waits for any active SPI DMA transaction to complete before
 * performing blocking SPI register reads.
 *
 * The wait prevents the polling SPI transaction from competing with
 * an already active DMA transaction on the same SPI peripheral.
 */
void imu_debug_dump_regs(void)
{
    HAL_NVIC_DisableIRQ(IMU_INT_EXTI_IRQn);

    /*
     * Disabling the EXTI interrupt prevents new acquisitions from
     * starting, but does not cancel a DMA transfer that is already
     * active. Wait for the SPI peripheral to become idle before
     * switching to blocking register access.
     */
    uint32_t wait_start = HAL_GetTick();

    while (HAL_SPI_GetState(s_hspi) != HAL_SPI_STATE_READY)
    {
        /*
         * Use a bounded wait so that a hardware fault cannot leave
         * the diagnostic routine blocked indefinitely.
         */
        if (HAL_GetTick() - wait_start > 5)
        {
            break;
        }
    }

    printf("--- IMU register dump ---\r\n");

    printf("WHO_AM_I     (0x75) = 0x%02X\r\n", imu_read_reg(MPU6500_WHO_AM_I_REG));

    printf("PWR_MGMT_1   (0x6B) = 0x%02X\r\n", imu_read_reg(MPU6500_PWR_MGMT_1));

    printf("INT_PIN_CFG  (0x37) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_PIN_CFG));

    printf("INT_ENABLE   (0x38) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_ENABLE));

    printf("INT_STATUS   (0x3A) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_STATUS));

    printf("-------------------------\r\n");

    HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);
}

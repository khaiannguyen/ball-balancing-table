# 05 — MPU6500 IMU: SPI2 DMA, Data-Ready Interrupt, and 1 kHz Kalman Fusion

> Based on the actual implementation: `App/bsp/imu_mpu6500.c/h`, `App/bsp/exti_dispatch.c`, `App/utils/imu_fusion.c/h`, and `App/tasks/task_imu_fusion.c`.

---

## 1. Sampling Pipeline: EXTI → DMA → Callback → Task Notification

```text
MPU6500 INT pin (data-ready, 1 kHz)
        │  rising edge
        ▼
EXTI → HAL_GPIO_EXTI_Callback()
        │  centralized dispatcher
        ▼
imu_read_dma_start()
        │  SPI-busy guard
        │  D-Cache clean
        │  SPI2 DMA start
        ▼
HAL_SPI_TxRxCpltCallback()
        │  D-Cache invalidate
        │  raw accel/gyro parsing
        ▼
vTaskNotifyGiveFromISR()
        │
        ▼
ImuFusionTask
        │  Kalman filter
        ▼
IMU state published through seqlock
```

The complete acquisition path is event-driven. There is no periodic polling of the IMU data-ready condition:

```text
Hardware interrupt
      ↓
SPI DMA
      ↓
DMA completion callback
      ↓
RTOS task notification
      ↓
Kalman fusion
```

`exti_dispatch.c` is the centralized dispatcher for EXTI sources used by the IMU and six buttons. This avoids multiple definitions of the weak `HAL_GPIO_EXTI_Callback()` symbol and keeps interrupt routing in one place.

Task notification is preferred for this ISR-to-task handoff because the event represents a simple "new sample available" condition rather than ownership of a shared resource.

---

## 2. `imu_read_dma_start()` Owns the SPI-Busy Guard

The SPI-busy check is intentionally implemented inside the IMU driver:

```c
void imu_read_dma_start(void)
{
    if (HAL_SPI_GetState(s_hspi) != HAL_SPI_STATE_READY) {
        s_dma_start_busy_skip++;
        return;
    }

    s_dma_tx_buf[0] = MPU6500_ACCEL_XOUT_H | 0x80;

    SCB_CleanDCache_by_Addr(
        (uint32_t*)s_dma_tx_buf,
        IMU_DMA_BUF_LEN_ALIGNED);

    imu_cs_low();

    HAL_SPI_TransmitReceive_DMA(
        s_hspi,
        s_dma_tx_buf,
        s_dma_rx_buf,
        IMU_DMA_BUF_LEN);
}
```

The driver therefore remains safe regardless of which subsystem requests a DMA read in the future.

The D-Cache clean occurs immediately before starting DMA because the CPU has just written the SPI command into the transmit buffer and the DMA controller reads physical RAM rather than the CPU cache.

This follows the H723 memory rule documented in `02-stm32h723-cache-memory.md`:

```text
CPU writes TX buffer
        ↓
Clean D-Cache
        ↓
DMA reads RAM
        ↓
SPI transfer
```

---

## 3. Correct DMA Completion Callback

The project uses:

```c
HAL_SPI_TransmitReceive_DMA()
```

which is a full-duplex SPI transfer.

Therefore, the completion callback is:

```c
HAL_SPI_TxRxCpltCallback()
```

not:

```c
HAL_SPI_RxCpltCallback()
```

The latter corresponds to an RX-only DMA operation.

This distinction caused a real debugging issue: the SPI DMA transfer could start successfully, but the sample counter remained at zero because the callback implemented at that time was not the callback invoked by the HAL for a full-duplex transfer.

The lesson is simple:

> **The HAL callback must match the exact transfer API used to start the DMA transaction.**

---

## 4. SPI Error Handling

The IMU driver explicitly implements `HAL_SPI_ErrorCallback()`:

```c
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        s_dma_error_count++;
        s_last_spi_error_code = HAL_SPI_GetError(hspi);
        imu_cs_high();
    }
}
```

This prevents HAL-level SPI failures from becoming silent failures.

A DMA transfer may enter the `BUSY` state successfully and still fail before reaching the normal completion callback because of an internal SPI error such as:

- Overrun;
- Mode Fault;
- CRC error;
- Framing error.

Without an application callback, the HAL weak default may provide no application-level diagnostic information.

The driver therefore exposes counters including:

```text
s_dma_start_attempts
s_dma_start_busy_skip
s_dma_start_fail
s_dma_error_count
s_last_spi_error_code
```

through `imu_debug_print_dma_counters()`.

This turns an intermittent "DMA stopped" symptom into measurable failure categories.

---

## 5. MPU6500 Initialization and Write-Verify

`imu_mpu6500_init()` uses a write-then-read verification helper:

```c
imu_write_reg_verify()
```

The purpose is to verify that the device actually accepted the configuration rather than assuming that `HAL_OK` from the SPI transaction means the sensor register contains the intended value.

Three important initialization issues were addressed.

### 5.1 Disable the IMU interrupt before initialization

The IMU EXTI interrupt is disabled before SPI configuration.

This is necessary because CubeMX enables the EXTI interrupt during GPIO initialization. If the MPU6500 remains powered while the MCU is reset through a debugger, the sensor can still generate data-ready pulses using its previous configuration.

An interrupt arriving while `imu_mpu6500_init()` is performing blocking SPI transactions can cause:

```text
Initialization SPI transaction
        +
EXTI-triggered DMA transaction
        ↓
Shared SPI/CS/MISO contention
        ↓
HAL_BUSY / corrupted data
```

The initialization sequence therefore disables the interrupt first, completes configuration, clears any pending EXTI flag, and only then re-enables the interrupt.

### 5.2 Correct `INT_ENABLE` bit

The data-ready interrupt requires:

```text
INT_ENABLE (0x38)
RAW_RDY_EN = bit 0
```

An earlier implementation wrote `0x02`, which targeted bit 1 rather than the data-ready enable bit. The SPI write itself could succeed while the expected interrupt was never generated.

### 5.3 Enable the DLPF before relying on `SMPLRT_DIV`

The MPU6500 configuration uses:

```text
CONFIG      = 0x03
SMPLRT_DIV  = 0x00
```

The DLPF configuration is important because the internal sampling relationship changes when the digital low-pass filter is bypassed.

The project previously observed approximately 4 kHz sampling even though the intended rate was 1 kHz. The configuration was corrected so that the internal sample rate and subsequent divider produce the intended 1 kHz data-ready rate.

---

## 6. Full-Scale Configuration and Unit Conversion

The sensor is configured for:

```text
Gyroscope     ±500 dps
Accelerometer ±4 g
```

The corresponding conversion constants are:

```c
#define GYRO_LSB_PER_DPS  65.5f
#define ACCEL_LSB_PER_G   8192.0f
```

The important design issue is that the sensor register configuration and conversion constants are maintained in separate source locations.

If one is changed without the other:

```text
Sensor register
      │
      ▼
different physical scale
      │
      ├── raw data still looks valid
      ├── angles still move in the correct direction
      └── magnitude is wrong
```

There may be no compiler error and no obvious runtime fault.

The MPU6500 scale table is therefore kept close to the conversion code:

| Gyro FS | LSB/(°/s) | Accel FS | LSB/g |
|---|---:|---|---:|
| ±250 dps | 131.0 | ±2 g | 16384.0 |
| ±500 dps | 65.5 | ±4 g | 8192.0 |
| ±1000 dps | 32.8 | ±8 g | 4096.0 |
| ±2000 dps | 16.4 | ±16 g | 2048.0 |

This makes the relationship between hardware configuration and numerical conversion explicit.

---

## 7. Two-State Kalman Filter for Roll and Pitch

The firmware uses a lightweight 2-state Kalman filter:

```text
state = [angle, gyro_bias]
```

Two independent instances are used:

```text
Kalman Roll
Kalman Pitch
```

A full 6-state EKF including yaw is unnecessary for this mechanical system because yaw is not required for ball balancing.

The state model is:

```text
angle_dot = gyro_rate - bias
bias_dot  = 0
```

The accelerometer provides an independent angle measurement:

```c
float roll_acc =
    atan2f(ay, az) * (180.0f / M_PI);

float pitch_acc =
    atan2f(-ax, sqrtf(ay * ay + az * az))
    * (180.0f / M_PI);
```

The filter then performs:

```text
Gyroscope
   │
   ▼
Predict
   │
   ▼
Accelerometer angle
   │
   ▼
Update
   │
   ▼
Roll / Pitch estimate
```

The filter runs at the intended 1 kHz sample period using:

```c
#define IMU_SAMPLE_DT_S  0.001f
```

A fixed sample period is used rather than deriving `dt` from a FreeRTOS tick because a 1 ms RTOS tick does not provide additional timing resolution for this 1 kHz sensor path.

---

## 8. Kalman Tuning Parameters

The filter exposes three principal tuning parameters:

| Parameter | Increasing it means | Current value |
|---|---|---:|
| `Q_angle` | Greater trust in gyro propagation | `0.0008` |
| `Q_bias` | Different bias process-noise assumption | `0.003` |
| `R_measure` | Less trust in accelerometer measurement | `0.0156 / 0.0148` |

These values are documented as starting points rather than universal optimum values.

Final tuning should be based on measured mechanical behavior by comparing the estimated platform angle against an independent reference while changing platform orientation and observing servo-induced vibration.

---

## 9. Accelerometer Trust Gating During Vibration

Accelerometer-based tilt estimation assumes that the dominant acceleration is gravity.

During strong servo-induced vibration or dynamic motion, that assumption becomes less reliable.

The firmware therefore evaluates the acceleration magnitude:

```c
float accel_norm =
    sqrtf(ax * ax + ay * ay + az * az);

bool trust_accel =
    (accel_norm > 0.8f) &&
    (accel_norm < 1.2f);
```

When the magnitude is outside this range, the accelerometer measurement is treated as unreliable.

Instead of creating a separate `predict_only()` implementation, the existing Kalman update path temporarily uses a very large measurement noise:

```text
R_measure → 1.0e6
```

The resulting Kalman gain approaches zero, which makes the update effectively equivalent to a predict-only step without introducing a second algorithmic code path.

This keeps the filter implementation compact while preserving the intended behavior during vibration.

---

## 10. Publishing the Fused State

The fused state:

```text
roll
pitch
vroll
vpitch
```

is published through:

```c
imu_state_write()
```

using the project's seqlock mechanism.

The ownership model is:

```text
ImuFusionTask
     │
     │ single writer @ 1 kHz
     ▼
 IMU state
     │
     ├── ControlLoopTask
     ├── CanTxTask
     └── DisplayTask
```

A mutex would allow lower-priority readers to block the 1 kHz writer.

The seqlock allows the writer to update without blocking while readers retry if they detect that a write overlapped their snapshot.

This matches the shared-state synchronization policy described in `03-freertos-design.md`.

---

## 11. Bounded Task Notification Wait

`ImuFusionTask` waits for new samples using:

```c
ulTaskNotifyTake()
```

with a 20 ms timeout:

```c
#define IMU_NOTIFY_TIMEOUT_MS  20u
```

The wait is intentionally bounded rather than infinite.

If an interrupt or DMA transaction is temporarily lost:

```text
No notification
      │
      ▼
20 ms timeout
      │
      ├── task continues
      ├── previous raw state remains available
      └── watchdog can still observe task progress
```

This prevents a temporary sensor-path problem from permanently blocking the task.

It does not replace the system-level failsafe. If sensor data becomes invalid for long enough to affect safe control, the higher-level control and safety logic remains responsible for deciding how the actuator system should respond.

---

## 12. 32-Byte DMA Buffer Alignment

The SPI DMA buffers are aligned to a complete Cortex-M7 cache line:

```c
__attribute__((aligned(32)))
```

and padded to:

```c
IMU_DMA_BUF_LEN_ALIGNED = 32
```

even though the actual sensor transaction is only 15 bytes:

```text
1 byte command
+
14 bytes accel / temperature / gyro data
```

The padding is intentional.

A 32-byte cache-line-aligned buffer avoids sharing the DMA buffer with unrelated neighboring variables when `SCB_InvalidateDCache_by_Addr()` is used after DMA reception.

The full memory-system rationale is documented in:

`02-stm32h723-cache-memory.md`

---

## 13. Engineering Summary

The IMU subsystem combines:

```text
MPU6500 data-ready interrupt
        ↓
Centralized EXTI dispatch
        ↓
SPI2 full-duplex DMA
        ↓
Cortex-M7 D-Cache maintenance
        ↓
DMA completion callback
        ↓
FreeRTOS task notification
        ↓
2-state Kalman fusion
        ↓
Seqlock-published state
```

The design keeps the high-rate acquisition path event-driven and avoids polling, while explicitly handling the two Cortex-M7 issues most likely to cause subtle DMA failures:

- cache coherency;
- DMA buffer placement/alignment.

The sensor driver also contains explicit error counters and register write verification so that hardware or HAL failures remain observable during integration rather than appearing only as missing or stale control data.

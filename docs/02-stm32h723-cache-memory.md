# 02 — STM32H723 Memory, Cache, DMA, and Clock Architecture

> This document describes the memory, cache, DMA, MPU, FPU, and clock-tree considerations of the STM32H723 firmware. The analysis is based on the actual Ball Balancing Table firmware, including `main.c`, `system_stm32h7xx.c`, linker scripts, `imu_mpu6500.c`, and `tft_service.c`.

---

## 1. Why H7 Memory Architecture Matters

The STM32H723 uses a Cortex-M7 core and differs architecturally from common STM32F1/F4 devices in two areas that are particularly important for DMA-based embedded systems:

1. **I-Cache and D-Cache**
2. **Multiple memory domains with different bus and DMA access characteristics**

The HAL APIs can look almost identical across STM32 families:

```c
HAL_SPI_TransmitReceive_DMA()
```

but the memory system underneath is significantly different.

For this project, these differences directly affected the reliability of:

- SPI2 + DMA for the IMU;
- SPI1 + DMA for the TFT;
- shared memory buffers;
- real-time sensor acquisition.

---

## 2. Cortex-M7 Cache Architecture

The H723 provides:

- I-Cache for instruction fetch;
- D-Cache for data access;
- hardware floating-point support.

The project enables both caches at startup:

```c
int main(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    MPU_Config();
    HAL_Init();
    ...
}
```

The cache configuration is performed before the remainder of system initialization so that subsequent code execution benefits from the enabled instruction and data caches.

The important consequence is that CPU-visible memory and DMA-visible physical RAM are not automatically coherent.

---

## 3. Cache–DMA Coherency

### 3.1 The problem

The Cortex-M7 D-Cache may contain a newer or older copy of data than the physical RAM accessed by DMA.

Two transfer directions therefore need different handling.

### CPU → DMA

If the CPU writes a buffer and DMA subsequently reads it:

```text
CPU
 │
 ▼
D-Cache
 │
 │ newest data may still be cached
 ▼
RAM
 │
 ▼
DMA
```

DMA does not read through the CPU D-Cache. The buffer must therefore be cleaned before starting the transfer.

### DMA → CPU

If DMA writes a buffer and the CPU subsequently reads it:

```text
DMA
 │
 ▼
RAM
 │
 │ CPU may still have an old cached copy
 ▼
D-Cache
 │
 ▼
CPU
```

The old cache line must be invalidated before the CPU reads the new data.

The practical rule is:

> **Clean before DMA reads CPU-produced data. Invalidate before CPU reads DMA-produced data.**

---

## 4. IMU DMA Implementation

The IMU uses SPI2 with DMA.

The DMA buffers are explicitly aligned and sized to a complete Cortex-M7 cache line:

```c
#define IMU_DMA_BUF_LEN_ALIGNED 32

__attribute__((aligned(32)))
static uint8_t s_dma_tx_buf[IMU_DMA_BUF_LEN_ALIGNED];

__attribute__((aligned(32)))
static uint8_t s_dma_rx_buf[IMU_DMA_BUF_LEN_ALIGNED];
```

### Before starting a transfer

```c
s_dma_tx_buf[0] = MPU6500_ACCEL_XOUT_H | 0x80;

SCB_CleanDCache_by_Addr(
    (uint32_t*)s_dma_tx_buf,
    IMU_DMA_BUF_LEN_ALIGNED);

HAL_SPI_TransmitReceive_DMA(
    s_hspi,
    s_dma_tx_buf,
    s_dma_rx_buf,
    IMU_DMA_BUF_LEN);
```

The clean operation ensures that DMA reads the same data that the CPU just wrote.

### After DMA completion

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        imu_cs_high();

        SCB_InvalidateDCache_by_Addr(
            (uint32_t*)s_dma_rx_buf,
            IMU_DMA_BUF_LEN_ALIGNED);

        ...
    }
}
```

The invalidate operation ensures that subsequent CPU reads obtain the data written by DMA.

---

## 5. Why the Buffer Is 32-Byte Aligned

The Cortex-M7 cache line is 32 bytes.

Cache maintenance operates on cache lines rather than arbitrary logical application objects.

Using a small, unaligned buffer can create an unintended relationship with neighboring variables:

```text
Cache line
┌────────────────────────────────────────┐
│ DMA buffer │ neighboring variable      │
└────────────────────────────────────────┘
```

Invalidating the cache line for the DMA buffer could therefore affect the cached state of an unrelated variable.

The project avoids this by:

- aligning DMA buffers to 32 bytes;
- allocating a complete 32-byte region for the transfer buffer.

This is a low-level detail that is easy to miss when porting DMA code from an MCU without data cache.

---

## 6. TFT DMA and Cache Maintenance

The TFT is a write-only display path:

```text
CPU → RAM buffer → DMA → SPI1 → ILI9225
```

There is no application data read-back path from the TFT.

Therefore, the relevant cache operation is:

```c
SCB_CleanDCache_by_Addr(...)
```

before DMA transmits a buffer.

The project uses the same coherency principle for text sprites and graphics buffers in `tft_service.c`.

Unlike the IMU receive buffer, no cache invalidation is required for the TFT transmit path because the TFT does not DMA-write data back into system memory.

---

## 7. STM32H723 Memory Domains

The linker configuration defines multiple memory regions:

```text
ITCMRAM   0x00000000   64 KB
DTCMRAM   0x20000000  128 KB
RAM_D1    0x24000000  320 KB
RAM_D2    0x30000000   32 KB
RAM_D3    0x38000000   16 KB
```

These regions are not interchangeable.

| Region | Primary characteristic | DMA1/DMA2 access | D-Cache |
|---|---|---:|---:|
| ITCM | Low-latency CPU instruction/data access | No | No |
| DTCM | Very low-latency CPU data access | No | No |
| RAM_D1 | AXI SRAM, high bandwidth | Yes | Yes |
| RAM_D2 | AHB SRAM / peripheral domain | Yes | Yes |
| RAM_D3 | D3 domain, lower-power / backup-related use | Not for the DMA path used here | Configuration-dependent |

The key point for this project is:

> **A memory region can be fast for the CPU and still be unsuitable for DMA.**

In particular, DTCM is attractive for low-latency data but is not accessible by the DMA1/DMA2 path used for the IMU SPI transfers.

---

## 8. Linker Script and DMA Placement

The project contains two linker configurations:

```text
STM32H723ZGTX_FLASH.ld
STM32H723ZGTX_RAM.ld
```

The Flash build places normal `.data` and `.bss` storage in `RAM_D1`:

```text
.data → RAM_D1
.bss  → RAM_D1
```

This is compatible with the DMA path used by the SPI2/IMU implementation.

The RAM-debug configuration places normal application data in DTCM:

```text
.data → DTCMRAM
.bss  → DTCMRAM
```

That difference is important because a DMA buffer that works in the Flash build can become invalid for DMA when the debug configuration is changed.

A DMA API returning `HAL_OK` does not necessarily prove that the DMA controller can access the target memory correctly. The descriptor/configuration can be accepted while the actual transfer later fails or produces invalid data.

---

## 9. Recommended Explicit DMA Section

A more robust long-term design is to define a dedicated DMA section in both linker scripts:

```c
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_dma_rx_buf[IMU_DMA_BUF_LEN_ALIGNED];
```

The linker scripts can then explicitly map `.dma_buffer` to a DMA-accessible SRAM region.

This removes an accidental dependency on the default placement of global variables.

The intended architecture becomes:

```text
Application variables
        │
        ├── normal RAM placement
        │
        ▼
     RAM_D1/D2

DMA buffers
        │
        ├── explicit section
        │
        ▼
DMA-accessible SRAM
```

This is preferable when the project contains multiple build configurations.

---

## 10. MPU Configuration

The project configures the Cortex-M7 MPU.

Its purpose here is not task-level memory isolation. Instead, it provides controlled access attributes for the processor address space and helps prevent unwanted speculative accesses to unused external-memory regions.

The configuration establishes a large no-access region with selected subregions enabled for:

- internal Flash / ITCM;
- internal SRAM;
- peripherals;
- system-control space.

This is particularly relevant on Cortex-M7 devices because speculative access behavior can expose unexpected memory-system faults when unused memory regions are not configured appropriately.

The MPU should therefore be viewed as part of the **memory-system reliability configuration**, not as an application-level protection mechanism between FreeRTOS tasks.

---

## 11. FPU Configuration

The STM32H723 provides a single-precision floating-point unit.

The project enables CP10/CP11 access conditionally:

```c
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2)) |
                   (3UL << (11*2)));
#endif
```

The firmware uses `float` for high-rate numerical processing such as IMU fusion:

```text
Kalman filter
atan2f()
sqrtf()
```

This is appropriate for the H723 because single-precision operations can use the hardware FPU directly.

Using `double` would not provide the same hardware acceleration on this MCU and would introduce unnecessary software-emulated floating-point operations for this application.

The project build configuration uses:

```text
FPv5-SP-D16
```

which matches the single-precision FPU available on the H723.

---

## 12. FPU Context Switching

FPU context handling is provided through the ARM Cortex-M7 FreeRTOS port and hardware lazy stacking.

This is distinct from application-level floating-point enable flags.

The relevant architectural separation is:

```text
Compiler / MCU configuration
        │
        ▼
FPU availability

FreeRTOS Cortex-M7 port
        │
        ▼
FPU context preservation
```

Therefore, the kernel's task-switching behavior should be evaluated from the selected FreeRTOS ARM port rather than assuming that one application configuration macro alone determines FPU context handling.

---

## 13. Clock Tree

The H723 clock architecture is more complex than many STM32F1/F4 designs because it provides multiple PLLs and multiple peripheral clock paths.

The project configures PLL1 with:

```c
PLLM = 1;
PLLN = 50;
PLLP = 1;
PLLQ = 5;
PLLR = 2;
```

The board is operated at approximately:

```text
SYSCLK       400 MHz
SystemCoreClock 400 MHz
HCLK         200 MHz
PCLK1        100 MHz
PCLK2        100 MHz
FDCAN clock   80 MHz
```

The firmware intentionally runs at 400 MHz rather than the H723 maximum frequency.

---

## 14. Runtime Clock Verification

A key engineering practice in this project is to verify peripheral clocks at runtime rather than relying exclusively on hand calculations.

For example:

```c
uint32_t fdcan_clk_hz =
    HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);

printf(
    "FDCAN kernel clock = %lu Hz\r\n",
    (unsigned long)fdcan_clk_hz);
```

During debugging, the relevant RCC registers were also inspected to verify that the source-selection fields matched the intended configuration.

This is important on H7 because several prescaler stages can exist between a PLL output and a peripheral clock.

The practical rule is:

> **Use the configured register values as the source of truth for configuration, and verify the resulting clock through the HAL/runtime measurement when peripheral timing matters.**

This is particularly important for:

- FDCAN bitrate configuration;
- SPI timing;
- timer frequencies;
- RTOS timing assumptions.

---

## 15. Clock Verification Results

| Clock domain | Runtime value |
|---|---:|
| SYSCLK | **400 MHz** |
| SystemCoreClock | **400 MHz** |
| HCLK | **200 MHz** |
| PCLK1 | **100 MHz** |
| PCLK2 | **100 MHz** |
| FDCAN kernel clock | **80 MHz** |

The values are consistent with the configured bus prescalers and the measured/runtime-reported system configuration.

---

## 16. H7 vs F1/F4 — Practical Comparison

| Feature | STM32F1/F4 | STM32H723 |
|---|---|---|
| Instruction/data cache | Generally absent on the referenced F1/F4 class | I-Cache + D-Cache |
| RAM architecture | Simpler SRAM model | ITCM / DTCM / D1 / D2 / D3 domains |
| DMA memory constraints | Simpler | DMA accessibility depends on memory region |
| DMA/cache coherency | Usually not applicable | Must be explicitly handled |
| FPU | Device-dependent | Single-precision FPU |
| PLL architecture | Simpler | Multiple PLLs and peripheral clock paths |
| MPU / speculative-access considerations | Less significant | Relevant to Cortex-M7 memory configuration |
| Maximum CPU frequency | Lower | Up to 550 MHz device rating; project runs at 400 MHz |

---

## 17. Engineering Lessons

### 17.1 DMA success is not only an API question

A call such as:

```c
HAL_SPI_TransmitReceive_DMA(...)
```

returning successfully does not guarantee that the buffer is located in a memory region accessible by the DMA controller.

Memory placement must be considered together with peripheral/DMA configuration.

### 17.2 Cache coherency is part of the driver

On a Cortex-M7, a DMA driver is incomplete if it ignores the CPU cache.

The data path must explicitly define:

```text
CPU → DMA : Clean
DMA → CPU : Invalidate
```

### 17.3 Buffer alignment is part of correctness

Cache-line alignment is not merely a performance optimization when cache maintenance is performed on DMA buffers. It prevents unrelated variables from sharing cache lines with the buffer being invalidated or cleaned.

### 17.4 Linker configuration is part of embedded software architecture

Changing from a Flash linker script to a RAM-debug linker script can change where buffers are physically located.

Therefore, DMA buffer placement should be explicit rather than an accidental consequence of default `.bss` placement.

### 17.5 Runtime verification is valuable on complex clock trees

For timing-sensitive peripherals, runtime clock measurements and register inspection provide stronger evidence than relying only on manual derivation.

---

## 18. Final Memory/Data-Path Model

The relevant data paths can be summarized as:

### IMU

```text
CPU
 │
 ▼
TX buffer
 │
 │ Clean D-Cache
 ▼
RAM_D1 / DMA-accessible SRAM
 │
 ▼
DMA1
 │
 ▼
SPI2
 │
 ▼
MPU6500
 │
 ▼
SPI2 + DMA1
 │
 ▼
RX buffer
 │
 │ Invalidate D-Cache
 ▼
CPU
```

### TFT

```text
CPU
 │
 ▼
Graphics / text buffer
 │
 │ Clean D-Cache
 ▼
DMA-accessible RAM
 │
 ▼
DMA
 │
 ▼
SPI1
 │
 ▼
ILI9225
```

The central design constraint is that **CPU cache state, physical RAM placement, DMA accessibility, and peripheral configuration must be considered as one system**.

That constraint is fundamental to reliable high-speed DMA operation on the STM32H723.

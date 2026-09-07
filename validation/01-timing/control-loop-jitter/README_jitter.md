# Control-loop jitter — STM32H723 (section 1.1)

## Summary

> Control loop jitter: 4.12 µs std (osDelay) → 8.67 µs std (vTaskDelayUntil), N=2048 cycles, Balance mode — vTaskDelayUntil did not reduce short-window jitter in this run (both well under 0.1% of the 10ms period); see `validation/01-timing/control-loop-jitter/`.

## Background

`task_control_loop.c` used `osDelay(CONTROL_LOOP_PERIOD_MS)` at 3 call sites in the main loop of ControlLoopTask, while `task_watchdog.c` and `task_can_tx.c` both used `vTaskDelayUntil()` (periodic, absolute-time, non-drifting). This is the most critical task in the system (it directly generates servo commands), so this bug was prioritized for a fix, with before/after measurements.

## Measurement method

- **Tool:** DWT cycle counter (Cortex-M7, CPU 400MHz → `time_us = cycles / 400`).
- **Sampling point:** start of the `for(;;)` loop body of ControlLoopTask, recorded into a static ring buffer `s_period_buf[2048]` (`uint32_t`, unit: cycles between two consecutive loop entries).
- **Measurement conditions:** Balance mode, under real load (servo + IMU + CAN running simultaneously), not measured at idle.
- **Sample count N:** 2048 (buffer fully filled — confirmed `s_idx == 2048` before dumping, to avoid dumping a partially-filled buffer).
- **Data capture method:** halted via debugger (CubeIDE) exactly when the buffer was full, exported the `s_period_buf` memory region via Memory View → RAW Binary (`.bin`, 8192 bytes = 2048 × 4 bytes), then parsed with a Python script (`analyze.py` / `analyze_after.py`) to compute mean/std/min/max in µs and export to CSV.
- **No UART/printf in the hot path** — `retarget.c` uses `HAL_UART_Transmit(..., HAL_MAX_DELAY)` (genuinely blocking, byte-by-byte), which could corrupt the measurement if accidentally logged inside the loop.

## Results

| | before (`osDelay`) | after (`vTaskDelayUntil`) |
|---|---|---|
| N | 2048 | 2048 |
| mean (µs) | 9999.91 | 9999.81 |
| **std (µs)** | **4.12** | **8.67** |
| min (µs) | 9815.57 | 9608.17 |
| max (µs) | 10004.78 | 10004.69 |

Raw data files: `before/before_raw.bin`, `before/before_summary.csv`, `after/after_raw.bin`, `after/after_summary.csv`.

## Assessment

`vTaskDelayUntil` **did not improve jitter** within this short measurement window (~20 seconds, N=2048) — the measured std actually **increased** compared to `osDelay` (8.67 µs vs 4.12 µs), the opposite of the initial hypothesis.

This is a valid finding, not a measurement error: both std values are very small relative to the 10ms period (0.04% and 0.087% respectively), showing the system is stable in both versions over the measured interval — the absolute difference is only a few µs. The discrepancy may stem from the two measurement runs not having perfectly identical load conditions (measured in two separate debug sessions, separated in time); it's also possible that measurement-environment noise (cache write-back into RAM_D1, see the buffer note below) contributed more to this difference than the delay mechanism itself.

**On theoretical grounds, `vTaskDelayUntil` should still be kept** — its main benefit is protecting against **long-term accumulated drift**, an effect that requires a much longer run than this ~20-second measurement to manifest clearly in the instantaneous period's std. The current measurement is not sufficient to rule out that theoretical benefit; it only shows that, within a short window, both approaches are acceptable.

## Technical note on the measurement buffer

The `s_period_buf` buffer uses a plain static array (approach (a) in the plan), not forced into DTCM — because the project's current linker script (`STM32H723ZGTX_FLASH.ld`) does not define a `.dtcm` section in `SECTIONS{}`, so the `section(".dtcm")` attribute would not actually place the buffer in DTCM. The buffer resides in `RAM_D1` (AXI SRAM), which may be subject to write-back caching since `SCB_EnableDCache()` is called in `main.c` — a minor but acceptable noise source relative to the ms-scale jitter being measured.

## Operational notes (lessons learned while measuring)

- Must confirm `s_idx == 2048` via **Live Expressions** (no need to halt to view — CubeIDE updates this value while the board is running) **before** halting to dump. Some trial dumps that yielded N < 2048 (buffer not yet full) were discarded and not used in the results table above.
- Any leftover breakpoints (e.g., a breakpoint accidentally left in FreeRTOS's `tasks.c`) must be cleared before Resume, otherwise the program may halt partway at the idle task instead of running continuously until the buffer fills.
- After changing the code (`osDelay` → `vTaskDelayUntil`), the project must be rebuilt (not just Debug/Resume on the old binary) before taking the "after" measurement.

## Evidence

```
validation/01-timing/control-loop-jitter/
├── README.md
├── before/
│   ├── before_raw.bin
│   └── before_summary.csv
└── after/
    ├── after_raw.bin
    └── after_summary.csv
```

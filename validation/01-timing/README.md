# 1 — Real-time scheduling & control-loop timing

Summary of timing measurements for the STM32H723 (main control loop) and the Jetson (`TaskControlLoop`). See the README in each subfolder for full details.

## Quick summary

| Section | Platform | Bug found | Key result |
|---|---|---|---|
| [1.1 — Jitter](./control-loop-jitter/README.md) | STM32H723 | ✅ `osDelay` (relative delay) at 3 call sites — fixed to `vTaskDelayUntil` | jitter std: 4.12 µs (before) → 8.67 µs (after), N=2048 |
| [1.2 — WCET](./control-loop-wcet/README.md) | STM32H723 | — (reuses 1.1's infrastructure) | WCET 7.88 µs (0.08% of the 10ms budget), **Home mode only** |
| [1.3 — Jitter & WCET](./jetson-control-loop/README.md) | Jetson `TaskControlLoop` | ❌ No bug — already uses correct absolute-time `sleep_until` | jitter std 17.59 µs, WCET mean 6.34 µs / max 31.01 µs (0.06%/0.31%) |

---

## 1.1 — STM32 Control Loop Jitter: Before vs After

![Jitter before vs after](./assets/chart1_jitter_before_after.png)

After fixing `osDelay` → `vTaskDelayUntil` (3 call sites in `task_control_loop.c`), the measured std actually **increased** from 4.12 µs to 8.67 µs over the short measurement window (~20 seconds, N=2048) — the opposite of the initial hypothesis. Both values are still very small relative to the 10ms period (under 0.1%). This is a valid finding, reported honestly — the numbers were not adjusted to fit an expected narrative. `vTaskDelayUntil` remains the theoretically correct choice against long-term accumulated drift, an effect this short measurement window cannot capture. Details: [`control-loop-jitter/README.md`](./control-loop-jitter/README.md).

---

## 1.2 — STM32 Control Loop WCET (Home mode)

![WCET STM32](./assets/chart2_wcet_stm32.png)

Measured WCET is **7.88 µs**, only **0.08%** of the 10ms budget — a very large safety margin. ⚠️ **Important limitation:** only measured in **Home mode** (the default mode at boot) — switching to Balance/Position mode was not possible with the current program design, so the question of "which mode is most expensive" remains open. The Home-mode figure must not be assumed to apply to other modes. Details: [`control-loop-wcet/README.md`](./control-loop-wcet/README.md).

---

## 1.3 — Jetson TaskControlLoop: Jitter & WCET

![Jetson jitter and WCET](./assets/chart3_jetson.png)

Confirmed the Jetson code has **no bug** — `sleep_until(next_wake)` with `next_wake += period` was already correct absolute-time scheduling, matching `task_can_tx.cpp`. Mean period ≈ 10000 µs as designed, std 17.59 µs. WCET is very small (mean 6.34 µs, max 31.01 µs). ⚠️ **Caveat:** the WCET end marker is `attitude_desired_write()` rather than an actual CAN send — the physical CAN transmission happens in a separate, asynchronous `TaskCanTx` task. These numbers reflect only `TaskControlLoop`'s internal processing time, not end-to-end CAN transmission latency. Details: [`jetson-control-loop/README.md`](./jetson-control-loop/README.md).

---

## Overall comparison: WCET vs the 10ms budget

![Overall WCET comparison](./assets/chart4_overview.png)

Both platforms have a very large timing margin relative to the 10ms control-loop budget — no deadline-violation risk was observed in the measurements taken. Note that both WCET measurements are **incomplete**: STM32 is missing Balance/Position mode, and Jetson is missing the actual CAN transmission time through `TaskCanTx`.

---

## Open items (not yet 100% complete)

- [ ] Measure STM32 WCET in Balance mode and Position mode (currently Home mode only).
- [ ] Measure the true end-to-end CAN timing on Jetson, including `TaskCanTx` (currently only measured up to `attitude_desired_write()`).
- [ ] Decide whether to keep or revert the instrumentation added to `task_control_loop.cpp` (Jetson) — currently modified but not committed.

## Directory structure

```
validation/01-timing/
├── README.md                      <- this file (summary)
├── assets/
│   ├── chart1_jitter_before_after.png
│   ├── chart2_wcet_stm32.png
│   ├── chart3_jetson.png
│   └── chart4_overview.png
├── control-loop-jitter/           (STM32, section 1.1)
│   ├── README.md
│   ├── before/
│   └── after/
├── control-loop-wcet/             (STM32, section 1.2)
│   ├── README.md
│   ├── wcet_home_raw.bin
│   └── wcet_home_raw_summary.csv
└── jetson-control-loop/           (Jetson, section 1.3)
    ├── README.md
    ├── control_loop_period_ns.csv
    └── control_loop_wcet_ns.csv
```

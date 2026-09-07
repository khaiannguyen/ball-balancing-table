# Control-loop WCET — STM32H723 (section 1.2)

## Summary

> Control loop WCET: 7.88 µs (0.08% of 10ms budget), N=2048 cycles, Home mode (default mode at boot — Balance/Position could not be measured, see limitations below) — see `validation/01-timing/control-loop-wcet/`.

## Measurement method

Reused the DWT infrastructure from section 1.1, adding a dedicated buffer `s_exec_buf[2048]` and a pair of markers:
- **t0:** start of per-cycle processing in the `for(;;)` loop, right after the period-measurement code from 1.1.
- **t1:** immediately after the `servo_actuator_step()` call — placed at all 3 real call sites of this function in the code (the `state != STATE_RUN` branch, the `!setpoint_get` branch, and the main branch after `switch(sp.mode)`), so no cycle is missed regardless of system state.

`time_us = (t1 - t0) / 400` (CPU 400MHz). N = 2048 samples, confirmed `s_exec_idx == 2048` (buffer fully filled) before halting and dumping — same rigorous verification process as section 1.1, following the lessons learned there about partially-filled buffer dumps.

## Results

| | Home mode (default mode at boot) |
|---|---|
| N | 2048 |
| mean (µs) | 2.85 |
| **WCET = max (µs)** | **7.88** (0.08% of the 10ms budget) |
| min (µs) | 2.705 |
| std (µs) | 0.365 |

Raw data files: `wcet_home_raw.bin`, `wcet_home_raw_summary.csv`.

## Answering the 3 questions from section 1.2

**1. What percentage of the 10ms budget does the WCET consume?**
7.88 µs / 10000 µs ≈ **0.08%** — an extremely large margin, no risk of deadline violation in the measured mode.

**2. Which mode is the most expensive (Balance vs Position vs Home)?**
**Cannot be fully answered** — only **Home mode** was measured. See the Limitations section below.

**3. Are there any anomalously long `dt` values (multi-segment trajectory)?**
None observed — `max = 7.88 µs` is only slightly higher than `mean = 2.85 µs` (about 2.8×), and `std = 0.365 µs` is very small relative to the mean. No abnormally high spikes were seen in the N=2048 sample set in Home mode. It cannot be ruled out that multi-segment trajectory consumption only occurs in Position/Balance mode (not measurable here).

## Measurement limitations — important, read before using these figures

Only **Home mode** was measured (the default mode when the system boots, `OPMODE_HOME = 0`). Balance mode and Position mode **could not be measured** because the current program design did not allow switching modes under the actual operating conditions available at measurement time. As a result:

- The WCET figure of 7.88 µs / 0.08% **represents Home mode only** and cannot be assumed or extrapolated to Balance/Position.
- Home mode is likely the **least computationally expensive mode** (no closed-loop sensor-feedback PID as complex as Balance, no multi-segment trajectory interpolation as in Position) — so the true WCET of Balance/Position **could be significantly higher** than this figure. The Home-mode data must not be used to conclude the system is safe in every mode.
- The question of "which mode is most expensive" from section 1.2 **remains open**, and needs additional measurement once a way to switch modes becomes available (e.g., via a CAN command, a physical button, or a temporary code modification to force the mode during measurement).

## Evidence

```
validation/01-timing/control-loop-wcet/
├── README.md
├── wcet_home_raw.bin
└── wcet_home_raw_summary.csv
```

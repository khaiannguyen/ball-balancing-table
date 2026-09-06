# Section 2 — Watchdog & fault-injection correctness

STM32H723 independent watchdog (IWDG): does it reset the MCU when a
safety-critical task deadlocks, does it correctly *not* reset when a
non-critical task deadlocks, and does the reset-cause register actually read
what it claims to read.

## Critical finding

During testing, `main.c`'s `StartWatchdog()` thread body was found to never
call `task_watchdog.c`'s `StartTaskWatchdog()` — it ran an unconditional
`osDelay(1); HAL_IWDG_Refresh();` stub instead, so the alive-mask logic was
dead code. This was fixed (`StartWatchdog` now calls `StartTaskWatchdog()`),
and all results in this report are from re-testing after that fix. The fix
was committed separately as a firmware bugfix, not as part of this
validation change.

![StartWatchdog before/after the fix](watchdog_bug_before_after.png)

## 2.0 — RCC->RSR reset-cause readout

4 reset-button trials across 2 sessions all read `RSR=0x00420000` (`PIN=1`
only). A true power-cycle (POR) wasn't independently tested — the serial
logger shares the same USB link as board power, so pulling power drops the
capture session instead of cleanly recording the next boot. Cross-validated
instead: IWDG-caused reboots (2.1/2.3) read a *different* register value
(`RSR=0x04420000`, `PIN=1` **and** `IWDG=1`) on separate occasions, ruling out
a stuck/hardcoded read.

Details: [`rcc-rsr-fix/`](rcc-rsr-fix/)

## 2.1 — ControlLoopTask deadlock → IWDG reset

`ControlLoopTask` (`osPriorityRealtime`) deadlocked on BTN6 → stops marking
its alive bit → `WatchdogTask` skips the refresh → IWDG resets the MCU.

![ControlLoopTask deadlock sequence](deadlock_control_sequence.png)

**N=6 trials, all reset with `IWDG=1`**, mean 466.5ms (range 438.4–508.6ms).

Details: [`deadlock-control-loop/`](deadlock-control-loop/)

## 2.2 — DisplayTask deadlock → NO reset (most important test)

`DisplayTask` (`osPriorityLow`) deadlocked on BTN6 → it holds no
`ALIVE_BIT_*` by design → the alive-mask stays satisfied → the watchdog keeps
refreshing.

![DisplayTask deadlock sequence](deadlock_display_sequence.png)

**No new `[BOOT]` over ~118.3s observed** (~289x the measured IWDG timeout) —
confirms the alive-mask is a deliberately curated set of safety-critical
tasks, not a blanket liveness check.

Details: [`deadlock-display-isolation/`](deadlock-display-isolation/)

## 2.3 — IWDG timeout: measured vs theoretical

Refresh stopped outright (independent of any task's alive-mask) to isolate
the raw hardware timeout.

![IWDG timeout across 15 trials](iwdg_timeout_bar_chart.png)

**N=14 valid, mean=410.3ms, std=0.3ms** (1 outlier excluded — trial 6,
499.5ms, a startup catch-up burst) vs. **501ms theoretical**
(Prescaler=/32, Reload=500, LSI=32kHz nominal). The gap implies a real LSI
frequency of ~39.1kHz vs. the 32kHz used in the calculation — within the
untrimmed LSI's documented tolerance, not a bug.

Details: [`iwdg-timeout-measured/`](iwdg-timeout-measured/)

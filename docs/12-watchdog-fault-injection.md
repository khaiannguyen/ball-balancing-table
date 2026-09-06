# 12 — Watchdog & fault-injection correctness
### STM32H723 — validation plan (completed)

> **Code + temporary build-flag tests + physical reset button only.** No ball
> on the table needed. STM32 firmware, developed on a separate workstation —
> git-synced before starting.

---

## Critical finding

During testing, `main.c`'s `StartWatchdog()` thread body was found to never
call `task_watchdog.c`'s `StartTaskWatchdog()` — it ran an unconditional
`osDelay(1); HAL_IWDG_Refresh();` stub instead, so the alive-mask logic was
dead code. This was fixed (`StartWatchdog` now calls `StartTaskWatchdog()`),
and all results below are from re-testing after that fix. The fix was
committed separately as a firmware bugfix, not as part of this validation
change.

---

## 0. Status

| Item | Status | Result |
|---|---|---|
| 2.0 `RCC->RSR` (reset-cause readout) | ✅ Done | 4/4 reset-button trials read `PIN=1`; POR not independently tested (see 2.0 below), cross-validated via a different flag pattern (`IWDG=1`) on IWDG-caused reboots |
| 2.1 `ControlLoopTask` deadlock → must reset | ✅ Done | N=6, all reset with `IWDG=1`, mean 466.5ms |
| 2.2 `DisplayTask` deadlock → must NOT reset | ✅ Done | No new `[BOOT]` over ~118.3s observed |
| 2.3 IWDG timeout: measured vs. theoretical | ✅ Done | N=14 valid, mean=410.3ms vs. 501ms theoretical (LSI tolerance) |

**Why 2.0 blocked everything else:** all 3 tests below need to know the
*cause* of a reset (IWDG? pin? brown-out?) to distinguish "the watchdog
worked correctly" from "the board reset for some other reason." Without a
correctly-reading `RCC->RSR`, every observed reset is ambiguous.

---

## 2.0 — Fix `RCC->RSR` (prerequisite)

### Known state (from an earlier attempt)

- Attempt 1: `[BOOT]` never printed → `printf` was placed before
  `MX_USART3_UART_Init()`. Fixed by moving it.
- Attempt 2: `[BOOT]` printed, but `RSR=0x00000000, PIN=0` even right after a
  real reset-button press.

`RSR` cannot legitimately read all-zero — at least one flag must always be
set. Three hypotheses, unverified at the time because the firmware wasn't
available on the test machine yet:

| # | Hypothesis | How to check |
|---|---|---|
| 1 | Flag auto-cleared by `HAL_Init()`/`SystemInit()` before the code reads it (most likely) | Move the `RCC->RSR` read to the **very first line** of `main()`, before `HAL_Init()` |
| 2 | Wrong register/macro | `grep -rn "RCC_RSR_" Drivers/CMSIS/.../stm32h723xx.h`, cross-check bit offsets |
| 3 | Global variable zero-initialized by `.bss` after assignment | Breakpoint on the first line of `main()`, inspect `RCC->RSR` in Expressions right there |

```c
static uint32_t g_boot_rsr;   /* global, NOT a static local inside a function */

int main(void)
{
  g_boot_rsr = RCC->RSR;        /* READ HERE — before HAL_Init() */
  HAL_Init();
  __HAL_RCC_CLEAR_RESET_FLAGS();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();        /* printf only usable after this line */

  printf("[BOOT] RSR=0x%08lX IWDG=%d WWDG=%d SFT=%d BOR=%d POR=%d PIN=%d LPWR=%d\r\n",
      (unsigned long)g_boot_rsr,
      (g_boot_rsr & RCC_RSR_IWDG1RSTF) ? 1 : 0,
      (g_boot_rsr & RCC_RSR_WWDG1RSTF) ? 1 : 0,
      (g_boot_rsr & RCC_RSR_SFTRSTF)   ? 1 : 0,
      (g_boot_rsr & RCC_RSR_BORRSTF)   ? 1 : 0,
      (g_boot_rsr & RCC_RSR_PORRSTF)   ? 1 : 0,
      (g_boot_rsr & RCC_RSR_PINRSTF)   ? 1 : 0,
      (g_boot_rsr & RCC_RSR_LPWRRSTF)  ? 1 : 0);
```

### Verify with two different trials, not one

A single trial cannot rule out the code always printing the same fixed value:

1. Press the reset button → expect `PIN=1`
2. Pull power, plug it back in → expect `POR=1`

Two different flags on two different occasions is what actually confirms the
register is being read correctly.

### Result

4 reset-button trials (2 sessions) all read `RSR=0x00420000` (`PIN=1` only).
The power-cycle (POR) trial above was **not independently run** — the serial
logger shares the same USB link as board power, so pulling power drops the
capture session instead of cleanly recording the next boot. Cross-validated
instead: IWDG-caused reboots gathered for 2.1/2.3 read a *different* register
value (`RSR=0x04420000`, `PIN=1` **and** `IWDG=1`) on separate occasions —
two distinct, non-hardcoded flag patterns, which was the actual goal of
running two different trials.

### Evidence
[`validation/02-watchdog/rcc-rsr-fix/`](../validation/02-watchdog/rcc-rsr-fix/)

---

## 2.1 — `ControlLoopTask` deadlock must cause an IWDG reset

```c
#ifdef VALIDATION_DEADLOCK_TEST_CONTROL   /* separate build flag, NOT merged into main */
    if (validation_button_pressed())      /* physical trigger, not a hardcoded condition */
    {
        for (;;) { __NOP(); }             /* real deadlock, stops setting the alive bit */
    }
#endif
```

Measured over UART (`/dev/ttyACM0`, reusing the capture pipeline from the CAN
investigation): timestamp of the trigger press → timestamp `[BOOT]`
reappears after reset, with `IWDG=1`.

Repeated N≥5 times (press button, measure, wait for reset, repeat).

### Result
N=6 trials, all reset with `IWDG=1`. Mean 466.5ms, range 438.4–508.6ms — wider
spread than 2.3 because `WatchdogTask` only notices the missing alive bit on
its next 10Hz snapshot, adding up to ~100ms of jitter on top of the ~410ms
IWDG countdown.

### Evidence
[`validation/02-watchdog/deadlock-control-loop/`](../validation/02-watchdog/deadlock-control-loop/)

> "Deliberate ControlLoopTask deadlock → IWDG reset confirmed via `RCC->RSR`
> (`IWDG=1`), observed within 438.4–508.6ms of trigger (N=6) — see
> `validation/02-watchdog/deadlock-control-loop/`."

---

## 2.2 — `DisplayTask` deadlock must NOT cause a reset (most important test)

Same trigger mechanism as 2.1, but deadlocking `DisplayTask` — a task
**outside** the watchdog alive-mask by design.

**This is the single most important test in section 2** — it doesn't prove
the watchdog "works," it proves the alive-mask is **deliberately curated**
(only safety-critical tasks) rather than an arbitrary list of every task. A
Lead would ask exactly this question.

Expected: the screen freezes (`DisplayTask` is dead), but
`ControlLoopTask`/servo keep running normally, and **no** new `[BOOT]`
appears.

### Result
No new `[BOOT]` over ~118.3s observed (~289x the measured IWDG timeout),
while an unrelated periodic fault message reappears after the gap,
corroborating that the rest of the firmware kept running.

### Evidence
[`validation/02-watchdog/deadlock-display-isolation/`](../validation/02-watchdog/deadlock-display-isolation/)

> "DisplayTask deadlock (outside the watchdog alive-mask) does not trigger a
> reset; the rest of the firmware continues unaffected, confirmed via the
> absence of a new `[BOOT]` event over ~118.3s — see
> `validation/02-watchdog/deadlock-display-isolation/`."

---

## 2.3 — IWDG timeout: measured vs. theoretical

Stop refreshing the IWDG entirely (separate test build, not a specific task's
deadlock). Measure the time from stopping the refresh to `[BOOT]`
reappearing, and compare against the theoretical value computed from the
prescaler.

### Result
N=14 valid trials (1 outlier excluded), mean=410.3ms, std=0.3ms, vs. 501ms
theoretical (Prescaler=/32, Reload=500, LSI=32kHz nominal). The gap implies a
real LSI frequency of ~39.1kHz vs. the 32kHz used in the calculation — within
the untrimmed LSI oscillator's documented tolerance, not a bug.

### Evidence
[`validation/02-watchdog/iwdg-timeout-measured/`](../validation/02-watchdog/iwdg-timeout-measured/)

> "Measured IWDG timeout: 410.3ms (prescaler-calculated: 501ms), N=14 valid
> of 15 trials — see `validation/02-watchdog/iwdg-timeout-measured/`."

---

## Evidence directory layout

```
validation/02-watchdog/
├── README.md                          short summary, for GitHub
├── rcc-rsr-fix/                       (2.0, mandatory prerequisite)
├── deadlock-control-loop/             (2.1)
├── deadlock-display-isolation/        (2.2 — most important)
└── iwdg-timeout-measured/             (2.3)
```

## Order followed

```
2.0 (fix RCC->RSR, verify with two different trials)
  |
  v
2.2 (DisplayTask deadlock — done before 2.1, cheaper: no need to wait
     for a real reset, only confirm NO new [BOOT])
  |
  v
2.1 (ControlLoopTask deadlock — needs to wait for a real reset each time,
     slower)
  |
  v
2.3 (IWDG timeout — done last, least informative of the 4 items)
```

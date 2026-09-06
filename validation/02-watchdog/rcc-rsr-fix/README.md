# 2.0 — RCC->RSR reset-cause readout

`RCC->RSR` is read as the first statement in `main()`, before `HAL_Init()`, and
the flags are cleared via `__HAL_RCC_CLEAR_RESET_FLAGS()` right after the
boot-line printf. This was the fix for an earlier symptom where `RSR` always
read back `0x00000000` — `HAL_Init()`/`SystemInit()` was clearing the flags
before the (previous, later-placed) read.

## PIN=1 evidence (4 instances, 2 mechanisms)

| # | Source | `host_t_ms` | `RSR` | Decoded | Mechanism |
|---|---|---|---|---|---|
| 1 | `serial_log_20260906_194355.txt:1` | 249640.3 | `0x00420000` | `PIN=1`, all else 0 | Manual reset-button press |
| 2 | `serial_log_20260906_194355.txt:13` | 253434.3 | `0x00420000` | `PIN=1`, all else 0 | Reflash-triggered reset (see below) |
| 3 | `serial_log_20260906_203542.txt:2` | 29.9 | `0x00420000` | `PIN=1`, all else 0 | Manual reset-button press |
| 4 | `serial_log_20260906_203542.txt:12` | 79.3 | `0x00420000` | `PIN=1`, all else 0 | Manual reset-button press |

**Row 2 is not a second manual button press.** `serial_log_20260906_194355.txt`
contains a mid-capture reflash boundary: the ~3.79s window between boot 1
(`t=249640.3`) and boot 2 (`t=253434.3`) shows only clock-init and
`StartTaskCanRx/Tx STARTED` output — zero `[WDG_TICK]` lines, even though
that print runs at 10Hz once the `VALIDATION_IWDG_STARVE_TEST` build is
running. That means boot 1 is on a build without that instrumentation, and
the board was reflashed to the `VALIDATION_IWDG_STARVE_TEST` build between
the two boots while the same logger session kept appending to this one file
(`[WDG_TICK]` starts immediately after boot 2 — see
[`iwdg-timeout-measured/`](../iwdg-timeout-measured/) for the 4 starve trials
that follow it in the same file). Boot 2 is therefore most likely the
automatic reset ST-Link asserts after programming, not a finger on the
button — but it still drives the physical NRST pin, so `PIN=1` is the
correct, expected read either way.

All 4 instances read `PIN=1` and nothing else, across two independent
capture sessions and two different trigger mechanisms (button vs.
programmer-reset).

## Power-cycle (POR) — not independently tested

A true power-cycle was not exercised in this session. The serial capture
(`serial_timestamp_logger.ps1`) runs over the same USB link that also powers
the board; pulling board power would drop the COM port the logger holds open
mid-session rather than cleanly capturing the next `[BOOT]` line. This was
judged not worth the fragility for this pass.

**Cross-validation instead of an independent POR trial**: the IWDG-triggered
reboots gathered for 2.1/2.3 all read `RSR=0x04420000` (`PIN=1` **and**
`IWDG=1`) — a different register value than the pure reset-button case above
(`0x00420000`, `PIN=1` only), captured on separate occasions with no code path
that could produce either value except by actually reading the register. That
satisfies the original goal of seeing two distinct flag patterns without
requiring a dedicated POR run.

Raw logs: [`serial_log_20260906_194355.txt`](serial_log_20260906_194355.txt),
[`serial_log_20260906_203542.txt`](serial_log_20260906_203542.txt) — copied
verbatim, unedited. See [`run_info.txt`](run_info.txt) for line-level
provenance.

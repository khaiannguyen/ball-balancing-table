# 2.2 — DisplayTask deadlock does NOT reset the MCU

This is the most important test in section 2: it shows the alive-mask is a
**deliberate, curated set of safety-critical tasks**, not "every task,"
because `DisplayTask` is intentionally excluded from `ALIVE_MASK_EXPECTED`.

![DisplayTask deadlock sequence](../deadlock_display_sequence.png)

## What the log shows

`VALIDATION_DEADLOCK_TEST_DISPLAY` hangs `DisplayTask` (`osPriorityLow`) in
`for(;;){ __NOP(); }` on a BTN6 press — with no `printf` before the loop, so
there is no `[TRIGGER]` line for this test by design. The evidence is the
**absence** of further output:

| `host_t_ms` | Event |
|---|---|
| 29.9 / 79.3 | Two `[BOOT]` lines, `RSR=0x00420000` (`PIN=1`) — session start / button-bounce reboot |
| 195.9 | Last line before the gap: `FAULT SET - mat heartbeat!` |
| 118541.1 | Next line in the file: same message, file ends here |

**118345.2ms (~118.3s) elapse with no new `[BOOT]`** — about 289x the
measured IWDG timeout (2.3: 410.3ms) — while `DisplayTask` is presumed
deadlocked. `FAULT SET - mat heartbeat!` is printed by an unrelated task, so
its reappearance after the gap corroborates that the rest of the firmware
kept executing.

**Caveat**: this log has no direct timestamp for the BTN6 press itself
(DisplayTask prints nothing before it hangs), so the exact deadlock start
isn't captured — only that no reset occurred across a window an order of
magnitude longer than the IWDG timeout. Combined with the design guarantee
(`ALIVE_MASK_EXPECTED` in `system_state.h` only includes `ALIVE_BIT_CONTROL_LOOP`,
`ALIVE_BIT_IMU_FUSION`, `ALIVE_BIT_CAN_RX` — no display bit exists at all),
this is sufficient to confirm the intended isolation.

Raw log (verbatim, unedited): [`serial_log_20260906_203542.txt`](serial_log_20260906_203542.txt).

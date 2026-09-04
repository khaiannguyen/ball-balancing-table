# CAN bus STM32H723 <-> Jetson: periodic fault-flap — fixed

125kbps CAN bus between STM32H723 (control) and Jetson (vision/control) periodically
loses sync, `stm32_ok` oscillates continuously 1↔0. Root cause **isolated**: the
CAN_H/CAN_L wire pair was not twisted, causing cross-coupled noise — twisting the wire
is the single change that wipes out the error; `sjw=16` is kept as an additional safety
margin but its individual contribution **has not been independently verified**.

## Elimination chain (in the exact order verified)

| Hypothesis | Result | Evidence |
|---|---|---|
| Intermittent physical contact (loose connector/solder joint) | **[RULED OUT]** | Wiggle test: error rate does not correlate with hand manipulation, and even accelerates during the final 40s of hands-off (34→36→46→65 errors/10s) — exactly the counter-test the test itself was designed to use to rule out this hypothesis |
| Bitrate mismatch between the 2 nodes | **[RULED OUT]** | Reading `FDCAN1->NBTP` directly via SWD (hotplug, no reset): the real STM32 runs 125000 bps, sample-point 0.875 — matches Jetson exactly |
| `AutoRetransmission` | **[RULED OUT, was already correct]** | SWD read of `FDCAN1->CCCR.DAR=1` → already DISABLEd on the real chip |
| default `sjw` → `sjw=16` | **Tried but NOT sufficient alone** | With the CAN_H/CAN_L wire pair not twisted, `sjw=16` alone still did not stop the errors (STM32 kept repeating error-passive/near-bus-off cycles per SWD, independent of whether the app was running) |
| **CAN_H/CAN_L wire pair not twisted → cross-coupled noise/EMI** | **[ROOT CAUSE — ISOLATED]** | Isolating the two variables, with `sjw=16` held fixed across both trials: wire not twisted → still errors; wire twisted → 0 errors, 600s soak with the real app, confirmed on both sides (see table below). Twisting the wire is the change that made the difference |

**Why the wiggle test does not contradict the final conclusion**: the wiggle test ruled
out one specific type of physical fault — **loose contact** (touching/wiggling the wire
does not cause errors to spike then stop when let go). An untwisted wire is a
**different** type of physical fault — passive cross-coupled noise/EMI between two wires
run in parallel without a twist, which does not change when the wire is touched, so it
falls outside what a test designed to find loose contact can detect. The two conclusions
do not contradict each other, they are just two different types of fault.

**`sjw=16`**: still kept in `scripts/can_up.sh` and `scripts/can0.service` as an
additional safety margin (no harm — still `<= phase-seg2=25`, does not affect the
sample-point). Its individual contribution (whether it is necessary alongside the
twisted wire, or just redundant) **has not been isolated independently** — see the "Not
yet done" section below.

## Mandatory operating rule

> **Never leave `can0` up with no process reading the socket** (the real app, or at
> minimum `candump`).

STM32 can fall into a repeating bus-off cycle due to `AckError` when nobody is listening
on the Jetson side — this phenomenon is **invisible to the Jetson-side error counters**
(the netlink `berr-counter` does not increase because Jetson is not attempting to
receive/send anything at all). This is an **operating precaution, not a fixed bug** —
emphasized clearly so nobody misreads it as a technical fix.

## Final verification: twisted wire + sjw=16, 600s soak

Measured 2026-09-04 22:02:39 → 22:12:39 (+07), `scripts/run_soak.sh 600
validation/03-can/twisted-pair-sjw16-verified`, the real app (`balance_ball_main`)
running throughout, the CAN_H/CAN_L wire pair kept in its twisted state, `can0` kept at
`sjw 16` (nothing changed for this measurement). Measured on both sides at the same
time — lesson from earlier investigation: TX errors on the Jetson side previously did
not correspond 1-1 to the STM32 side, and STM32 bus-off was previously invisible to
Jetson, so a single side cannot be relied on alone.

| Source | Metric | Result |
|---|---|---|
| `app.log` (Jetson, 3028 lines) | `FAULT` | 0 |
| `app.log` | `stm32_ok=0` | 0 |
| `app.log` | `[CAN_ERR]` | 0 |
| `candump.log` (169640 frames, 600s) | error frames (error-flag-shaped ID) | 0 — only 6 valid IDs (0x100-0x104, 0x1FF) |
| `candump.log` | gap >100ms | 0 |
| `canstats.log` (121 samples × 5s) | `can state` other than `ERROR-ACTIVE` | 0/121 |
| `canstats.log` | `berr-counter` other than 0 | 0/121 |
| `canstats.log` | `re-started/bus-errors/arbit-lost/error-warn/error-pass/bus-off` | 0, unchanged across all 121 samples |
| `canstats.log` | RX/TX `errors`/`dropped` (netdevice) | 0/0 |
| `stm32_uart.log` | `drop`/`busoff` (cumulative) | held steady at `4417`/`711` for the whole 600s → delta=0 |
| `stm32_uart.log` | `[BOOT]` (MCU reboot) | 0 |
| **SWD** `FDCAN1->ECR`/`PSR` (`stm32_swd_poll.csv`, 1180 samples × 500ms ≈ 590s, no reset) | TEC/REC | 0/0 throughout |
| SWD | EP/EW/BO (`PSR.EP/EW/BO`) | 0/0/0 throughout |
| SWD | `PSR.LEC` | only `NoError`/`NoChange` — not a single `StuffError/FormError/AckError/Bit1Error/Bit0Error/CRCError` |

Absolutely clean on both sides at the same time, measured independently by 3 different
methods (Jetson netlink, STM32 UART self-report, SWD reading STM32 registers directly)
— not dependent on a single data source.

![Error rate before/after twisting CAN wire](error_rate_before_after.png)

The "before" baseline is taken from the final 40s of the wiggle test (wire not twisted,
hands fully off, see the "Not yet done" section — the original file is no longer kept
in the repo, the figures are quoted verbatim: 34/36/46/65 errors/10s, accelerating).
"After" is the entire 600s just measured, flat at 0 throughout.

## Not yet done

- **The individual contribution of `sjw=16` has not been isolated independently.** Have
  not yet tried the twisted wire + default sjw (removing the `sjw` parameter from
  `ip link`, letting the driver pick its own default) to see whether the error comes
  back. Currently keeping `sjw=16` as a precautionary safety margin, not as a fix
  confirmed to be necessary.
- No evidence yet of continuous operation over many hours/across many power-cycle
  cycles — 600s is evidence ~15 times longer than the original error-acceleration
  window (40s), but is not yet a long-term test.
- The cause of the one STM32 self-reboot (t+374s, a very early 900s soak in the
  investigation) is still undetermined — it did not recur in any measurement afterward.

# WARNING: the source in this directory does NOT match the firmware actually running on the real chip

Written 2026-09-04, after investigating a recurring CAN bus fault between STM32H723
and Jetson (full process: `../docs/` and `validation/03-can/` on the `balance_ball`
repo side).

## Fact confirmed via SWD (reading live registers over ST-Link, no build/flash)

The STM32H723 chip actually running on the Nucleo-H723ZG board (confirmed via the
board's `NODE_H723ZG` MSC drive) has:

```
FDCAN1->NBTP: NominalPrescaler=40, NominalTimeSeg1=13, NominalTimeSeg2=2  -> 125000 bps, sample-point 0.875
FDCAN1->CCCR: DAR=1 -> AutoRetransmission = DISABLE
```

**The source in `Core/Src/main.c` in this directory (and on GitHub
`khaiannguyen/ball-balancing-table`, branch `main`, every commit/branch/stash checked)
instead reads:**

```c
hfdcan1.Init.NominalPrescaler = 10;   // -> 500000 bps if built/flashed from here, NOT 125000
hfdcan1.Init.AutoRetransmission = ENABLE;   // DIFFERENT from the DAR=1 (DISABLE) actually running
```

With `NominalPrescaler=10`, `NominalTimeSeg1=13`, `NominalTimeSeg2=2` and an FDCAN
kernel clock of 80MHz (measured via SWD, not assumed): bitrate = 80e6/(10×16) =
**500000 bps** — off by 4x from Jetson (`can0` runs at 125000 bps, see
`../jetson-vision-control/scripts/can_up.sh`).

## Why this is dangerous

If someone builds + flashes the STM32H723 from this very directory (or from the
pushed GitHub version, which currently looks identical), the board will run at the
wrong bitrate and reproduce exactly the type of CAN fault that took many days to
investigate (see `validation/03-can/README.md` on the `balance_ball` side for the full
account — the final root cause of the fault turned out to be the untwisted CAN_H/CAN_L
wire pair, causing crosstalk/EMI, NOT the bitrate; `sjw=16` is kept as an additional
safety margin but its own contribution was never verified in isolation — but if this
version is flashed by mistake there would be an additional real bitrate fault stacked
on top).

## Which version is the correct one?

The version actually running on the chip (`NominalPrescaler=40`,
`AutoRetransmission=DISABLE`) **cannot be found anywhere on this Jetson machine** —
not as a file, not as a git object (every commit/branch/stash/reflog checked), not as
a build artifact (`.elf`/`.hex`). It almost certainly only exists on the machine that
was last used to build + flash it with STM32CubeIDE. **Before building/flashing again
from this directory, find and cross-check against the project on that machine first.**

## To do (not done here — note only, `main.c` not self-edited)

1. Find the real STM32CubeIDE project (the machine that did the last flash), get the
   correct `NominalPrescaler`/`AutoRetransmission` from it.
2. Update `Core/Src/main.c` here to match, or — better — rebuild from that CubeIDE
   machine, read back via SWD once more to confirm the match, then sync the source
   into git.
3. Commit + push, delete this warning file once the source matches.

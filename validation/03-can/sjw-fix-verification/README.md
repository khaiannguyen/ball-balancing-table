# SJW=16 fix verification — 300s soak after persisting the fix

Date measured: 2026-09-04, right after persisting `sjw 16` into
`scripts/can0.service` (live: `/etc/systemd/system/can0.service`) + `scripts/can_up.sh`.
Purpose: confirm NUMERICALLY whether `sjw=16` was enough to stop the fault (not a
feeling/one short trial), with a duration much longer than the old error baseline so
as not to "miss" the error-rate-increasing-over-time phenomenon seen in the wiggle
test.

## OLD error baseline (before the fix) — reference

From [`../physical-fault-localization/README.md`](../physical-fault-localization/README.md),
step h (hands off completely, 40s, touching NOTHING) of the wiggle test:

| Window | error_pass increase |
|---|---|
| 140-150s | +34 |
| 150-161s | +36 |
| 161-172s | +46 (converted/10s) |
| 172-180s | +65 (converted/10s) |

The error rate **increases steadily** even with nobody touching the bus — this is
precisely the "bad" baseline to compare against.

## Re-measured AFTER persisting sjw=16

Method: a clean reset (`modprobe -r mttcan && modprobe mttcan`, confirming
`berr tx=0 rx=0` before measuring — same methodology as `wiggle_monitor.sh`/
`run_soak.sh`), `systemctl restart can0.service` (using the exact unit with the fix
persisted), then sampling `ip -details -statistics link show can0` every 0.5s for 300s
continuously — **longer than the old baseline (180s wiggle test / 40s reference
window)**, touching nothing, not running the app (methodology identical to the wiggle
test, the only difference being no hand manipulation since it is no longer needed).

Raw log: [`soak_berr.csv`](soak_berr.csv) (555 lines, 299.7s).

| Window (30 windows × 10s) | error_pass increase |
|---|---|
| 0-300s (all 30 windows) | **0 in ALL 30 windows** |

All 555 samples: `can_state` is always `ERROR-ACTIVE` (not a single time falling into
`ERROR-WARNING`/`ERROR-PASSIVE`/`BUS-OFF`), `berr-counter tx=0 rx=0` unchanged
throughout the 300s, `error-warn`/`error-pass`/`bus-off` cumulative counters all = 0,
not increasing.

## Conclusion

| | Old baseline (40s, no touch) | After the sjw=16 fix (300s, no touch) |
|---|---|---|
| error_pass/10s | 34, 36, 46, 65 (increasing) | **0, 0, 0, ..., 0 (30/30 windows)** |
| can_state | fluctuating ERROR-WARNING/PASSIVE | fixed ERROR-ACTIVE |
| berr tx/rx | rx fluctuating 100-127 | **0/0 unchanged** |

The error disappears completely, not merely reduced — 300 continuous seconds (~7.5
times the 40s window that produced the 34-65 errors/10s baseline) with not a single
error event, not a coincidence from one short trial. **Later found insufficient
alone**: a follow-up isolation test (untwisted CAN cable + `sjw=16`) still faulted;
only twisting the cable fixed it. `sjw=16` is kept as an unverified safety margin, not
a confirmed root cause — see [`../README.md`](../README.md) for the final root-cause
conclusion.

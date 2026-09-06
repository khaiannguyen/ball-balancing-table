# TaskControlLoop timing measurement — results

Date: 2026-09-07
Binary: `balance_ball_main` (full system: camera + CAN + PID + watchdog + video, run on the actual
Jetson with `can0` up at 125 kbps and the camera attached). Run duration: 25 s (`timeout --signal=INT 25`),
launched under `sudo` so `TaskControlLoop` could actually get `SCHED_FIFO` priority 80 as coded.
No ball was on the table — `detected=0` the whole run, which is fine: `TaskControlLoop` still executes
its normal 10 ms cycle (PID reset path) regardless of detection state.

Both instrumentation buffers (2000 samples each) filled after ~4 s and were dumped once to CSV
(`static bool timing_dumped` guard), so there is no file I/O in the `SCHED_FIFO` hot path beyond
that single one-time dump.

## Note on the "CAN send" measurement point

`task_control_loop.cpp` does **not** call `CanTransport::send()` anywhere — confirmed by grepping the
file. `TaskControlLoop` only writes the PID output into `system_state().attitude_desired` (a seqlock);
the actual CAN transmission (`can_.send(CAN_ID_ATTITUDE_DESIRED, ...)`, frame `0x204`) happens in a
different task, `TaskCanTx` (`task_can_tx.cpp`), on its own 100 Hz cycle. So the WCET end marker (`t1`)
used here is placed immediately after the `attitude_desired_write(...)` call — the actual point where
this task hands its output off toward CAN. This is the only such hand-off in the file, so there was
only one call site to cover.

## Period (jitter) — gap between consecutive loop entries, target 10.000 ms

| Stat | Value (µs) |
|---|---|
| n | 2000 |
| mean | 9999.946 |
| std dev | 17.591 |
| min | 9785.273 |
| max | 10216.203 |

Mean sits almost exactly on the 10 ms nominal period (absolute-time `next_wake += period` scheduling
is working as intended, no drift). Worst-case jitter observed: -215 µs / +216 µs around nominal.

## WCET — loop-body execution time, start of loop to just after `attitude_desired_write()`

| Stat | Value (µs) |
|---|---|
| n | 2000 |
| mean | 6.336 |
| std dev | 1.144 |
| min | 1.600 |
| max | 31.009 |

## WCET vs. 10 ms budget

| | % of 10 ms budget |
|---|---|
| mean WCET | 0.0634% |
| max WCET (observed) | 0.310% |

The control loop's own compute is a negligible fraction of its 10 ms period — plenty of slack, and the
observed max (31 µs) is nowhere near the budget, so no evidence of scheduling contention affecting this
task's execution time during this run.

## Files

- `control_loop_period_ns.csv` — raw period samples, nanoseconds (2000 rows + header)
- `control_loop_wcet_ns.csv` — raw WCET samples, nanoseconds (2000 rows + header)

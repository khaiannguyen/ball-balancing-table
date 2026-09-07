# Jetson `TaskControlLoop` jitter & WCET (section 1.3)

## Summary

> Jetson TaskControlLoop: already uses absolute-time sleep_until (no bug found) — jitter 17.59 µs std, WCET 6.34 µs mean / 31.01 µs max (0.06% mean / 0.31% max of 10ms budget), N=2000 cycles — see `validation/01-timing/jetson-control-loop/`.

## Confirmation from code review (before measuring)

Comparing `task_control_loop.cpp` against the reference pattern in `task_can_tx.cpp`: the main loop correctly uses `sleep_until(next_wake)` with `next_wake += period` — an absolute-time, self-correcting mechanism that does not accumulate drift. **No bug of the same kind as STM32** was found (STM32 used a relative `osDelay`, fixed in section 1.1). Therefore this section only measures once to confirm, with no before/after comparison.

## Measurement method

- **Tool:** `clock_gettime(CLOCK_MONOTONIC)`, in nanoseconds, converted to µs during analysis (`/1000`).
- **Period (jitter):** recorded at the start of the loop body, as the gap between two consecutive loop entries.
- **WCET:** `t0` at the start of the loop body, `t1` — see the important note below.
- **Buffer:** a `std::vector<uint64_t>` with `reserve(2000)` called before the real-time loop, dumped to CSV **exactly once** (using a `static bool` flag) after N=2000 samples were collected, with no other I/O in the hot path (the task runs under `SCHED_FIFO`).
- **Measurement conditions:** the full real system running (camera + CAN + PID + watchdog + video) for ~25 seconds, with `can0` up, **no ball needed on the table** — the camera views an empty frame (`detected=0` throughout), which is by design: `TaskControlLoop` still runs at the correct period regardless of whether a ball is present.

## ⚠️ Important note — deviation from the original spec for the WCET t1 marker

The original spec called for `t1` to be placed "right after the CAN send completes (`can_send` or equivalent)." However, upon reviewing the actual code, it was found that **`task_control_loop.cpp` does not call any CAN-send function directly** — it only writes the PID output into `system_state().attitude_desired` via the `attitude_desired_write()` function. The actual CAN transmission happens in a **separate task, `TaskCanTx`**, running asynchronously and independently of `TaskControlLoop`'s cycle.

For this reason, the `t1` marker in this measurement uses **`attitude_desired_write()`** — the only CAN hand-off point that exists in the `task_control_loop.cpp` file. This means:

- The WCET figures below reflect the **internal execution time of `TaskControlLoop`** (read sensors → compute PID → write setpoint) and **do not include** the actual physical CAN transmission time (that belongs to `TaskCanTx` and would need to be measured separately to get the true end-to-end WCET of the full CAN path).
- This is the most reasonable interpretation possible within the constraints of this file, but it differs from the word "can_send" in the original spec — noted explicitly here so future readers don't mistakenly assume the full CAN timing was measured.

## Results (N = 2000 samples)

| | Period (jitter) | WCET |
|---|---|---|
| mean (µs) | 9999.946 | 6.336 |
| std (µs) | 17.591 | 1.144 |
| min (µs) | 9785.273 | 1.600 |
| max (µs) | 10216.203 | 31.009 |
| % of the 10ms budget | — | mean 0.063%, max 0.310% |

## Assessment

- **Average period ≈ 10000 µs**, matching `CONTROL_LOOP_PERIOD_MS` exactly. `std = 17.59 µs` (0.18% of the period) — small, confirming that the `sleep_until` mechanism works as designed, with no accumulated drift over the 2000-cycle (~20 second) measurement window.
- **WCET is very small** relative to the budget: the mean consumes only 0.063%, and the worst case (max) only 0.31% of the 10ms budget — a very large margin for `TaskControlLoop`'s internal processing. It cannot be ruled out that the true end-to-end WCET (including `TaskCanTx`'s actual CAN send) is higher than this figure — see the note above.
- No jitter bug of the STM32 kind (`osDelay`) was found — this result confirms the initial expectation stated in the plan file, and no code change was needed.

## Evidence

```
validation/01-timing/jetson-control-loop/
├── README.md
├── control_loop_period_ns.csv
├── control_loop_wcet_ns.csv
└── control_loop_timing_summary.txt   (or .md, depending on what Claude Code saved it as)
```

The raw CSV files and summary are currently in `~/Downloads` on the Jetson — copy them into the correct directory above before committing.

## Code change note

`src/task_control_loop.cpp` was modified to add timing instrumentation (2 `std::vector` buffers, dumping to CSV once N=2000 samples are reached) — this change **has not been committed**. Consider either keeping this instrumentation in the code (possibly wrapped in `#ifdef VALIDATION_...`, similar to the `VALIDATION_DEADLOCK_TEST_CONTROL` style already used on STM32) to make re-measurement easy later, or removing it if this validation only needed a one-time measurement.

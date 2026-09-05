# Section 4 — End-to-end closed-loop performance

Two measurements of the ball-balancing table's closed-loop performance:
how accurately it tracks a moving setpoint (4.1), and how fast the system
reacts along the camera -> detection -> CAN -> control path (4.2).

## 4.1 — Tracking error

Reconstructed the trajectory controller's setpoint offline from a single
176.7s logged run (883 rows) and compared it against the measured ball
position, correcting for the closed-loop lag found via cross-correlation.

```
RMS error (lag-compensated): 28.0 mm   (91.7 mm without lag compensation)
N = 759 samples (detected=1 rows only)
Correlation: X = 0.96 @ lag~4 samples (~801ms), Y = 0.97 @ lag~5 samples (~1001ms)
```

**N=1 run — not repeated**, because ball detection is lighting-dependent and
a later run under different lighting would not be a valid repeated trial.

Details, method, and evidence: [`tracking-error-reconstructed/`](tracking-error-reconstructed/).

## 4.2 — End-to-end latency

Measured in two independently-instrumented segments and summed:

```
Camera pipeline delay (PTS vs steady_clock):        mean = 6.257ms  (N=250)
Injection -> 0x204 (CAN_ID_ATTITUDE_DESIRED) delay:  mean = 5.8ms, std = 2.8ms  (N=58)
                                                     -----------------------------
Total software response time:                       ~12.1ms
```

This is **software + CAN response time up to the point the PID output frame
appears on the CAN bus** — not the full physical settling time (real servo
motion + ball rolling inertia + camera re-detection), which is a different,
much larger timescale and is what the ~600-1000ms indirect estimate in 4.1
reflects. The two numbers are not meant to match.

Details, method, and evidence: [`synthetic-latency/`](synthetic-latency/).

# Section 4.2 — End-to-end latency (measured on real hardware)

Measured by splitting the path into two independently-instrumented segments
and summing them:

```
[Camera captures frame] --Segment A--> [Frame reaches detector] --Segment B--> [CAN -> PID -> servo PWM]
```

## Segment A — camera pipeline delay

`tools/camera_pts_latency_tool.cpp` + `CameraPipeline::enable_pts_latency_log()`
(`include/camera_pipeline.hpp`, `src/camera_pipeline.cpp`) read
`GST_BUFFER_PTS(buffer)` directly, convert it to an absolute pipeline-clock
time via `gst_element_get_base_time()`, and compare it against
`steady_clock::now()` at the moment the buffer reaches `handle_sample()`.
**Before trusting `delta_ms`, the code verifies the pipeline clock is
actually MONOTONIC** (comparing the order of magnitude of
`gst_clock_get_time()` against both `steady_clock` and `system_clock` at the
same instant in `open()`) — samples are only added to the statistics once
that match against `steady_clock` is confirmed.

Measured live on the real camera (no ball/physics needed), N=250 frames:

```
pipeline clock check => MONOTONIC (matches steady_clock, delta_ms is trustworthy)
delta_ms: mean=6.257  std=0.827  min=4.478  max=10.975  (N=250)
```

Evidence: `camera-pipeline-latency.log` (per-frame CSV:
`frame_index,pts_ns,base_time_ns,steady_now_ns,delta_ms`).

## Segment B — detection→CAN→control delay (synthetic ball injection)

Build flag `SYNTHETIC_FRAME_INJECTION` (`CMakeLists.txt`, default **OFF**,
never enabled in the main build). When enabled, `TaskCameraCapture`
(`src/task_camera_capture.cpp` in the `balance_ball` working repo — the
`jetson-vision-control` mirror in this repo predates this work and does not
yet contain this code) skips `nvarguscamerasrc` entirely and instead
generates a `cv::Mat` (one solid white circle, r=45px, on a dark background)
and `publish()`es it straight into the same `FrameBox` that
`TaskBallDetect` already consumes from — no parallel code path.

**Verified: the synthetic frame generator is rate-limited to the actual
configured camera cadence, not free-running.** It computes
`period = 1/fps` from the same `fps` argument passed to `TaskCameraCapture::start()`
(60 fps in this deployment, `cam.start(1280, 720, 60, 0)` in `main.cpp`) and
paces every publish with `next_wake += period; std::this_thread::sleep_until(next_wake)`.
So the ~12.1ms total below already reflects genuine per-frame-cadence-limited
timing — it is not an artificially fast, unbounded-loop number, and no
"add one camera frame period" correction is needed on top of it.

Cycle: 30 frames at the center of the frame -> 60 repeats of [hold center
>=1.0s -> jump +50px along the image X axis, log `steady_clock`+`system_clock`
to `inject_timestamps.log` -> hold offset >=1.6s -> return to center]. Run
live against the real STM32 (mode HOME, fixed setpoint (0,0)), with the
servos physically moving.

**Correlation uses only 0x204 (CAN_ID_ATTITUDE_DESIRED, the real PID output)
— never mixed with 0x200 (CAN_ID_BALL_POS, just Jetson echoing back the
position it read, near-instantaneous and not reflective of control
latency).**

`tools/match_inject_latency.py`'s direction check is a **self-consistency
filter, not an independent validation against a known-correct answer**: it
does not know in advance which sign the PID output should move in (that sign
depends on hardware polarity tuning — see the note in
`src/task_control_loop.cpp` about possibly negating the PID call). Instead,
for each of the 60 injections it finds the first post-jump `pitch_d` sample
that clears a noise threshold, records that crossing's sign, and takes the
sign that appears most often across all 60 trials as the "expected"
direction. Trials whose first crossing goes the other way are then dropped.
This is a legitimate way to reject noise/pre-existing oscillation that
happens to cross the threshold in the *minority* direction, but it derives
its own ground truth from the same 60-trial dataset it is filtering — it
cannot catch a scenario where the *majority* of trials share some other
common artifact (e.g. all 60 happening to react to something other than the
intended jump). It is a self-consistency check, not proof of correctness
against an external reference.

```
Majority direction (sign of pitch_d): -1
Matched majority direction: 58/60 (2 excluded)
Delta t (inject -> 0x204 appears on CAN bus): mean=5.8ms std=2.8ms N=58
```

Trials 58-59 were excluded: they coincide with a transient STM32 CAN
heartbeat fault (see app.log, validation/03-can/) that zeroed pitch_d via
the failsafe, not a matching-methodology issue.

Evidence: `inject_timestamps.log`, `candump.log` (full capture, `candump -tz can0`),
`can_response_0x200.log` / `can_response_0x204.log` (the two IDs split out
separately), `match_results.csv` (per-trial detail), `run_info.txt`
(`CANDUMP_START`, used to convert candump's relative timestamps to
wall-clock), `app.log` (full stdout of `balance_ball_main` during the run).

## Summing the two segments

```
Estimated latency (software + CAN response time only, up to the point the
0x204 frame appears on the bus — see note below) =
    6.257ms (Segment A) + 5.8ms (Segment B) ~= 12.1ms
```

## Limitations to state clearly

- Real sensor exposure/ISP time is not included (upper bound: 1 frame
  period, 16.7ms @ 60fps).
- Real ball rolling inertia is not included — this measures *software
  reaction capability*, not *full physical behavior*.
- The Segment B result measures up to the point the 0x204 frame **appears on
  the CAN bus**, **not** up to the point the STM32 actually updates servo
  PWM — the STM32 runs its own 100Hz control loop
  (`CONTROL_LOOP_PERIOD_MS=10`, `firmware-stm32h723/App/tasks/task_control_loop.c`),
  so add **up to ~10ms not measured here** if you need a literal "to the
  servo" number.
- **Important — why ~12.1ms is so different from the ~600-1000ms indirect
  estimate in section 4.1:** these are measuring two different things by
  design, not disagreeing about the same thing. Segments A and B only cover
  the **software response path up to the 0x204 frame reaching the CAN bus**
  (camera -> detection -> PID -> CAN TX). The ~600-1000ms figure in section
  4.1 is a cross-correlation estimate over the **full physical closed-loop
  settling time** — real servo mechanical motion, real ball rolling inertia,
  and the camera re-detecting the ball's new position after it has actually
  moved. Those physical dynamics are on a timescale roughly two orders of
  magnitude larger than the software/CAN path, so the ~12.1ms figure here
  and the ~600-1000ms figure in 4.1 are not directly comparable and this gap
  is expected, not a discrepancy to reconcile.
- 2/60 injections were excluded from the mean/std — see the identified cause
  (intermittent STM32/CAN heartbeat loss on the last two cycles) above and
  in `match_results.csv`.
- While building this, found and fixed one measurement-setup issue (not a
  control-logic bug): `CanTransport::open()` disables `CAN_RAW_LOOPBACK` by
  default (intentionally, so `TaskCanRx` does not receive back Jetson's own
  transmitted frames) — this also meant `candump` running locally on the
  Jetson itself could not see the 0x200/0x204 frames Jetson transmits (even
  though they genuinely go out on the bus and the STM32 receives them
  normally). Added an `enable_loopback` parameter (default `false`,
  preserving the main build's behavior) and enabled it only for the TX
  socket when building with `SYNTHETIC_FRAME_INJECTION` (`src/task_can_tx.cpp`)
  — `TaskCanRx` and the normal production build are unaffected.

## Deviations from the original plan (verified against the actual code)

- **`include/ball_detector.hpp` has no HSV threshold at all** — the original
  plan (and the 4.2 planning doc) assumed the detector used an "HSV
  threshold", but the real code uses a grayscale brightness threshold (after
  CLAHE) via `cfg_.white_threshold` (default 150, `main.cpp` sets 230). The
  synthetic frame uses a solid white circle (255,255,255) on a dark
  background (40,40,40) — above every threshold value used anywhere in the
  repo, not based on the incorrect HSV assumption.
- **There is no fixed pixel→mm convention in `ball_detector.hpp`** to look
  up the direction of a +50px jump — `pixel_to_world_mm()` does full
  projective geometry (undistort -> ray -> intersect the Z=0 plane) that
  depends entirely on `calib/extrinsic.yaml` loaded at runtime, not a fixed
  formula readable from the header. Instead of guessing the direction,
  `match_inject_latency.py` reads back the real Ballx value (via 0x200,
  used only to confirm direction — **never** for latency) and self-
  calibrates the expected direction by majority vote across the N trials
  (see the self-consistency caveat above).

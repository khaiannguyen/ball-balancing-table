#!/usr/bin/env python3
"""
Offline reconstruction of the BalanceTrajectoryController reference (Ballx_d,
Bally_d) from data.csv, for use when the firmware CSV logger did not record
the trajectory setpoint directly.

This is a line-by-line Python port of trajectory.hpp. It is driven by the
REAL elapsed time between consecutive log rows (timestamp_ms), split into
fine sub-steps (~10ms), rather than by a fixed nominal dt. This matters
because the integrative parts of the state machine (circle/figure-8 angle
accumulation) sum to exactly the real elapsed time regardless of how that
time is chopped into sub-steps, so this reconstruction is largely immune to
whatever control-loop period jitter existed on the real system -- as long as
the firmware itself calls update() with a real (not hardcoded) dt each cycle,
which is the standard and expected implementation.

Residual risk (must be stated honestly, not hidden):
  - If the real firmware instead calls update() with a HARDCODED constant dt
    (e.g. always 0.01f) regardless of actual elapsed loop time, then the
    firmware's own internal trajectory clock would itself be running at a
    slightly wrong rate whenever the loop drifted -- and this offline
    reconstruction (driven by true elapsed time) would then intentionally
    NOT match the firmware's (slightly wrong) internal state. This has not
    been verified by reading the call site in task_control_loop.cpp.
  - The TRANSIT sub-phase (0.2s duration) and the anchor point for reset()
    (assumed to be the very first log row) are the two places most sensitive
    to this assumption. Everything else (circle/figure8 angle integration)
    is essentially exact regardless.

Use the sanity-check output at the end of this script to judge whether the
reconstruction is self-consistent (does it complete a plausible number of
full sequence passes within the logged duration?) before trusting the
tracking-error numbers.
"""

import math
import sys
import pandas as pd

# ---------------------------------------------------------------------------
# Constants, ported verbatim from trajectory.hpp
# ---------------------------------------------------------------------------

PI = 3.14159265358979

# Hexagon waypoints (mm)
A = (120.25, -62.5)
B = (120.25, 62.5)
C = (0.0, 125.0)
D = (-108.25, 62.5)
E = (-108.25, -62.5)
F = (0.0, -125.0)
O = (0.0, 0.0)

SEQUENCE = [O, C, A, E, C, F, B, D, F, A, B, C, D, E, F]
N_POINTS = len(SEQUENCE)

HOLD_CENTER_S = 2.0
DWELL_S = 2.0
POST_FIG8_HOLD_S = 0.25
TRANSIT_S = 0.2
N_TRANSIT_POINTS = 3

CIRCLE_RADIUS = 120.0
CIRCLE_LAPS = 2
CIRCLE_SECONDS = 10.0
CIRCLE_TOTAL_ANGLE = 2.0 * PI * CIRCLE_LAPS
CIRCLE_ANGULAR_SPEED = CIRCLE_TOTAL_ANGLE / CIRCLE_SECONDS
CIRCLE_RAMP_S = 0.75
CIRCLE_START_ANGLE = -PI * 0.5

FIG8_START_ANGLE = PI * 0.5
FIG8_AMPLITUDE = 90.0
FIG8_LAPS = 4
FIG8_SECONDS = 24.0
FIG8_TOTAL_ANGLE = 2.0 * PI * FIG8_LAPS
FIG8_ANGULAR_SPEED = FIG8_TOTAL_ANGLE / FIG8_SECONDS
FIG8_RAMP_S = 0.30

SUB_STEP_S = 0.01  # nominal firmware control-loop period, used only to chop
                    # the real elapsed time into fine-grained update() calls


def clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


class TrajectoryState:
    """Direct port of BalanceTrajectoryController's private state."""

    def __init__(self):
        self.seq_index = 0
        self.sub_phase = "AT_POINT"  # AT_POINT | TRANSIT | CIRCLE | FIGURE8
        self.phase_elapsed_s = 0.0
        self.transit_step = 0
        self.post_figure8_hold = False
        self.circle_angle_traveled = 0.0
        self.figure8_angle_traveled = 0.0
        self.figure8_loop_angle = 0.0
        self.figure8_loop_index = 0

        # Diagnostics for the sanity check / repeatability grouping.
        self.full_passes_completed = 0

    def dwell_for(self, index):
        return HOLD_CENTER_S if index == 0 else DWELL_S

    def update(self, dt):
        self.phase_elapsed_s += dt
        next_index = (self.seq_index + 1) % N_POINTS

        if self.sub_phase == "AT_POINT":
            dwell = (
                POST_FIG8_HOLD_S
                if (self.seq_index == 0 and self.post_figure8_hold)
                else self.dwell_for(self.seq_index)
            )
            if self.phase_elapsed_s >= dwell:
                if self.seq_index == 0:
                    self.post_figure8_hold = False
                if self.seq_index == N_POINTS - 1:
                    self.sub_phase = "CIRCLE"
                    self.circle_angle_traveled = 0.0
                else:
                    self.sub_phase = "TRANSIT"
                self.phase_elapsed_s = 0.0
            return SEQUENCE[self.seq_index]

        if self.sub_phase == "TRANSIT":
            if self.phase_elapsed_s >= TRANSIT_S:
                self.phase_elapsed_s = 0.0
                self.transit_step += 1
                if self.transit_step >= N_TRANSIT_POINTS:
                    self.transit_step = 0
                    self.seq_index = next_index
                    self.sub_phase = "AT_POINT"
                    return SEQUENCE[self.seq_index]
            a = SEQUENCE[self.seq_index]
            b = SEQUENCE[next_index]
            frac = (self.transit_step + 1) / (N_TRANSIT_POINTS + 1)
            return (a[0] + (b[0] - a[0]) * frac, a[1] + (b[1] - a[1]) * frac)

        if self.sub_phase == "CIRCLE":
            time_in_circle = self.phase_elapsed_s
            self.circle_angle_traveled += CIRCLE_ANGULAR_SPEED * dt
            if self.circle_angle_traveled >= CIRCLE_TOTAL_ANGLE:
                self.circle_angle_traveled = CIRCLE_TOTAL_ANGLE
                self.sub_phase = "FIGURE8"
                self.phase_elapsed_s = 0.0
                self.figure8_angle_traveled = 0.0
                self.figure8_loop_angle = 0.0
                self.figure8_loop_index = 0
                return (0.0, 0.0)
            angle = CIRCLE_START_ANGLE + self.circle_angle_traveled
            time_remaining = CIRCLE_SECONDS - time_in_circle
            ramp_in = clamp01(time_in_circle / CIRCLE_RAMP_S)
            ramp_out = clamp01(time_remaining / CIRCLE_RAMP_S)
            r = CIRCLE_RADIUS * min(ramp_in, ramp_out)
            return (r * math.cos(angle), r * math.sin(angle))

        if self.sub_phase == "FIGURE8":
            self.figure8_loop_angle += FIG8_ANGULAR_SPEED * dt
            self.figure8_angle_traveled += FIG8_ANGULAR_SPEED * dt
            while self.figure8_loop_angle >= 2.0 * PI and self.figure8_loop_index < 4:
                self.figure8_loop_angle -= 2.0 * PI
                self.figure8_loop_index += 1
            if self.figure8_loop_index >= 4:
                self.seq_index = 0
                self.sub_phase = "AT_POINT"
                self.phase_elapsed_s = 0.0
                self.transit_step = 0
                self.post_figure8_hold = True
                self.figure8_angle_traveled = FIG8_TOTAL_ANGLE
                self.figure8_loop_angle = 0.0
                self.figure8_loop_index = 4
                self.full_passes_completed += 1
                return SEQUENCE[0]
            t = FIG8_START_ANGLE + self.figure8_loop_angle
            ramp_in = clamp01(self.phase_elapsed_s / FIG8_RAMP_S)
            amp = FIG8_AMPLITUDE * ramp_in
            s8, c8 = math.sin(t), math.cos(t)
            if self.figure8_loop_index < 2:
                return (amp * c8, amp * s8 * c8)
            else:
                return (amp * s8 * c8, amp * c8)

        return SEQUENCE[0]  # unreachable

    def phase_label(self):
        if self.sub_phase in ("AT_POINT", "TRANSIT"):
            names = ["O", "C", "A", "E", "C2", "F", "B", "D", "F2",
                     "A2", "B2", "C3", "D2", "E2", "F3"]
            return names[self.seq_index]
        return self.sub_phase


def find_reset_anchor(df):
    """
    trajectory.hpp's own reset() doc comment: "used when entering BALANCE
    mode or when the ball becomes available again after being lost."
    The reference (Ballx_d/Bally_d) can only meaningfully be compared to the
    measured ball position once tracking has actually started -- anchor
    reset() at the first row where detected transitions 0->1 (the first
    acquisition), not at the first row of the log.
    """
    det = df["detected"].to_numpy()
    for i in range(1, len(det)):
        if det[i - 1] == 0 and det[i] == 1:
            return i
    return 0  # detected from the very first row; no warm-up gap to skip


def reconstruct(csv_path):
    df = pd.read_csv(csv_path)
    ts_s = df["timestamp_ms"].to_numpy() / 1000.0

    anchor_i = find_reset_anchor(df)
    anchor_t = ts_s[anchor_i]

    state = TrajectoryState()
    bx_d = [None] * len(df)
    by_d = [None] * len(df)
    phase = [None] * len(df)

    # Before the anchor: ball not yet confirmed, trajectory has not started
    # advancing. Hold the neutral/O reference -- matches the failsafe/neutral
    # behavior expected while camera_ok/ball_on gating is not yet satisfied.
    for i in range(anchor_i):
        bx_d[i], by_d[i] = SEQUENCE[0]
        phase[i] = "PRE_DETECT"

    bx_d[anchor_i], by_d[anchor_i] = SEQUENCE[0]
    phase[anchor_i] = state.phase_label()

    for i in range(anchor_i + 1, len(ts_s)):
        real_dt = ts_s[i] - ts_s[i - 1]
        if real_dt <= 0:
            bx_d[i], by_d[i] = bx_d[i - 1], by_d[i - 1]
            phase[i] = state.phase_label()
            continue

        n_sub = max(1, round(real_dt / SUB_STEP_S))
        sub_dt = real_dt / n_sub

        wp = SEQUENCE[state.seq_index]
        for _ in range(n_sub):
            wp = state.update(sub_dt)

        bx_d[i], by_d[i] = wp
        phase[i] = state.phase_label()

    df["Ballx_d_reconstructed"] = bx_d
    df["Bally_d_reconstructed"] = by_d
    df["phase_reconstructed"] = phase

    return df, state, anchor_i, anchor_t


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "data.csv"
    df, final_state, anchor_i, anchor_t = reconstruct(csv_path)
    print(f"reset() anchor: row {anchor_i}, t={anchor_t:.2f}s "
          f"(first detected=1 sample)\n")

    out_path = "data_with_reference.csv"
    df.to_csv(out_path, index=False)

    total_s = df["timestamp_ms"].iloc[-1] / 1000.0

    # Only compute tracking error on rows where the ball was actually
    # detected -- comparing against (0,0) during camera warm-up would
    # artificially inflate/deflate the error with an undetected-ball
    # artifact, not a real tracking error.
    det = df[df["detected"] == 1].copy().reset_index(drop=True)
    det["err_x"] = det["Ballx"] - det["Ballx_d_reconstructed"]
    det["err_y"] = det["Bally"] - det["Bally_d_reconstructed"]
    det["err_mm"] = (det["err_x"] ** 2 + det["err_y"] ** 2) ** 0.5

    rms = (det["err_mm"] ** 2).mean() ** 0.5
    mean_err = det["err_mm"].mean()
    p95 = det["err_mm"].quantile(0.95)

    # Cross-correlation lag scan: measured ball position vs reconstructed
    # reference. A nonzero best-lag is expected in ANY real closed-loop
    # system (camera->processing->PID->servo->physical ball movement->
    # re-detection all take real time) and is itself a rough, indirect
    # estimate of end-to-end latency -- separate from, and not a substitute
    # for, a direct instrumented measurement per validation-plan item 10.4.
    import numpy as np
    bx = det["Ballx"].to_numpy()
    bxd = det["Ballx_d_reconstructed"].to_numpy()
    best_lag, best_corr = 0, -2.0
    for lag in range(-15, 16):
        if lag < 0:
            a, b = bx[-lag:], bxd[: len(bx) + lag]
        else:
            b, a = bxd[lag:], bx[: len(bxd) - lag]
        if len(a) < 30:
            continue
        c = float(np.corrcoef(a, b)[0, 1])
        if c > best_corr:
            best_corr, best_lag = c, lag

    approx_sample_period_s = (
        det["timestamp_ms"].diff().median() / 1000.0
    )
    approx_lag_s = best_lag * approx_sample_period_s

    lag_note = ""
    if best_lag != 0:
        # best_lag < 0 means Ball[i] correlates best with Reference[i-|lag|]
        # (the ball is a delayed copy of a past reference sample). Pair them
        # by shifting the REFERENCE forward by |best_lag| samples, keeping
        # the measured Ball series untouched -- do not shift Ball itself,
        # that pairs the wrong indices (an earlier mistake here initially
        # made the "compensated" error worse, not better).
        shifted = det.copy()
        shifted["Ballx_d_lagfix"] = (
            shifted["Ballx_d_reconstructed"].shift(-best_lag)
        )
        shifted["Bally_d_lagfix"] = (
            shifted["Bally_d_reconstructed"].shift(-best_lag)
        )
        shifted = shifted.dropna(subset=["Ballx_d_lagfix", "Bally_d_lagfix"])
        lag_err = (
            (shifted["Ballx"] - shifted["Ballx_d_lagfix"]) ** 2
            + (shifted["Bally"] - shifted["Bally_d_lagfix"]) ** 2
        ) ** 0.5
        lag_rms = (lag_err ** 2).mean() ** 0.5
        lag_note = (
            f"Lag-compensated RMS error (reference shifted by {-best_lag} "
            f"samples = {approx_lag_s * 1000:.0f} ms): {lag_rms:.1f} mm "
            f"(N={len(shifted)})"
        )

    print("=== Sanity check ===")
    print(f"Log duration:              {total_s:.1f} s")
    print(f"Full O->...->FIGURE8->O passes completed in reconstruction: "
          f"{final_state.full_passes_completed}")
    print(f"Final phase reached:       {final_state.phase_label()} "
          f"(sub_phase={final_state.sub_phase})")
    print()
    print("=== Tracking error (reconstructed reference vs measured ball), "
          "detected rows only ===")
    print(f"N samples used:  {len(det)} / {len(df)} total rows")
    print(f"RMS error (synchronous, no lag correction): {rms:.1f} mm")
    print(f"Mean error:      {mean_err:.1f} mm")
    print(f"95th pct error:  {p95:.1f} mm")
    print()
    print(f"Correlation(Ballx, Ballx_d) at lag 0: "
          f"{det['Ballx'].corr(det['Ballx_d_reconstructed']):.3f}")
    print(f"Best-fit lag: {best_lag} samples "
          f"(~{approx_lag_s * 1000:.0f} ms), corr={best_corr:.3f}")
    print(lag_note)
    print(
        "  -> a nonzero lag here is expected for any real closed-loop "
        "system and is a rough proxy for end-to-end latency (item 10.4), "
        "not a direct instrumented measurement."
    )
    print()
    print("Per-phase breakdown:")
    print(det.groupby("phase_reconstructed")["err_mm"]
          .agg(["count", "mean", "max"])
          .sort_values("mean", ascending=False)
          .to_string())
    print()
    print(f"Reconstructed CSV written to: {out_path}")


if __name__ == "__main__":
    main()

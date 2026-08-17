# 07 — Servo Trajectory Engine and Layered Actuator Safety

> Based on the actual implementation: `App/control/trajectory.c/h`, `App/control/servo_actuator.c/h`, `App/control/control_ball_common.c/h`, and `App/control/control_mode_balance.c`.

---

## 1. Three-Layer Actuation Architecture

The actuator path is deliberately divided into three independent layers:

```text
Jetson
Roll_d / Pitch_d / Height_d
        │
        ▼
Layer 3 — Trajectory Engine
        │
        │ Roll / Pitch / Height space
        │ acceleration-continuous replanning
        ▼
Layer 2 — Inverse Kinematics
        │
        │ Roll / Pitch / Height → S1 / S2 / S3 (µs)
        ▼
Layer 1 — Servo Actuator
        │
        │ slew-rate
        │ acceleration clamp
        │ deadband compensation
        │ hard limits
        ▼
servo_pwm.c → physical PWM
```

The trajectory and actuator limits intentionally coexist.

They operate in different domains:

- **Trajectory Engine:** physical platform coordinates such as degrees and millimeters.
- **Servo Actuator:** final PWM space after nonlinear inverse kinematics.

This separation is important because the second-order IK mapping can have different servo sensitivity at different platform orientations. A small change in Roll/Pitch does not necessarily produce the same PWM change for all operating points.

The Servo Actuator therefore remains the final hard safety boundary immediately before PWM.

Even if a trajectory calculation is incorrect, or another mode calls `servo_actuator_set_target()` directly, the physical PWM command remains constrained by the actuator limits.

---

## 2. Trajectory Engine — Acceleration-Continuous Replanning

### 2.1 Problem with the Original Approach

The original `trajectory_start()` assumed that every new trajectory began from:

```text
v0 = 0
```

This is reasonable for discrete movements such as:

```text
Home → target
Calibration → target
```

but it is not appropriate for Balance mode.

In Balance mode, the Jetson continuously updates:

```text
Roll_d
Pitch_d
Height_d
```

at a relatively high rate while the platform is already moving.

Restarting every trajectory from `v = 0` creates an artificial velocity discontinuity:

```text
Old trajectory:
      v ────────────►

New target arrives

Old implementation:
      v ──► 0 ──► accelerate again
```

The result is unnecessary servo motion whenever the target changes.

---

## 3. `trajectory_replan()` Preserves the Current Motion State

The revised API is:

```c
void trajectory_replan(trajectory_state_t *tr,
                       float x0,
                       float v0,
                       float a0,
                       float target,
                       float v_max,
                       float a_max);
```

The function receives the actual current trajectory state:

```text
x0 = current position
v0 = current velocity
a0 = current acceleration
```

The current velocity and acceleration are taken from the existing trajectory state after the previous `trajectory_update()`.

Three cases are handled.

### Case 1 — Already at the target

If the target is reached and:

```text
v0 ≈ 0
```

no new trajectory segments are required:

```text
seg_count = 0
```

### Case 2 — Current velocity is already in the required direction

The engine constructs:

```text
ACCEL → CRUISE → DECEL
```

starting from the current `(x0, v0)`.

If the remaining distance is too short to reach `v_max`, the cruise segment is automatically omitted and the profile becomes triangular.

The same ramp-building function is used for both:

```text
v0 = 0
v0 ≠ 0
```

rather than maintaining two separate implementations.

### Case 3 — Target direction reverses

If the new target requires movement opposite to the current velocity, the engine inserts an explicit braking segment:

```text
Current motion
      │
      ▼
BRAKE
      │
      ▼
v = 0
      │
      ▼
ACCEL → CRUISE → DECEL
```

The braking segment applies acceleration opposite to the current velocity until the trajectory reaches zero velocity at an intermediate point.

This prevents a sudden target reversal from directly commanding acceleration in the opposite direction.

---

## 4. Peak Velocity Calculation

The common ramp builder uses:

```c
float v_peak_sq = a_max * d + 0.5f * s * s;
float v_peak = sqrtf(v_peak_sq);

if (v_peak > v_max)
    v_peak = v_max;
```

This naturally produces either:

- a trapezoidal profile when `v_peak` reaches `v_max`;
- a triangular profile when the remaining distance is too short.

The resulting velocity profile avoids the artificial reset to zero that existed in the original point-to-point implementation.

The design target is **acceleration continuity at target updates**, not full jerk-limited S-curve motion.

---

## 5. `trajectory_update()` Handles Multiple Segment Boundaries

The trajectory update logic uses a loop:

```c
while (tr->seg_index < tr->seg_count && dt_left > 0.0f) {
    ...
}
```

rather than a single conditional.

This matters if one control-cycle `dt` becomes longer than the remaining duration of the current segment.

For example:

```text
Segment A ends
       │
       ▼
remaining dt
       │
       ▼
Segment B
```

The loop consumes the remaining time across multiple segments instead of discarding the unused portion.

Under normal 100 Hz operation this condition is uncommon, but handling it explicitly makes the trajectory engine robust against an unexpectedly long control-loop interval.

Once all segments are complete, the output is explicitly held at:

```text
(target, 0, 0)
```

rather than extrapolating beyond the final segment.

This avoids accumulated floating-point overshoot.

---

## 6. Why Full Jerk-Limited S-Curve Is Not Yet Used

The `trajectory_replan()` API already contains:

```c
a0
```

but the current Phase A implementation does not use it:

```c
(void)a0;
```

This leaves the API ready for a future jerk-limited Phase B without requiring another interface redesign.

A full S-curve implementation is intentionally deferred because the additional complexity is not currently justified by the mechanical response of the system.

The actuators are position servos with their own internal control behavior rather than pure velocity actuators. Acceleration-continuous trajectory generation therefore provides a useful improvement in smoothness without introducing the additional implementation and tuning risk of a full jerk-limited profile.

This is a deliberate scope decision rather than an accidental omission.

---

## 7. Servo Actuator Processing Order

`servo_actuator_step()` applies the actuator constraints in a fixed order:

```text
(a) desired velocity
        ↓
(b) anti-windup
        ↓
(c) slew-rate limit
        ↓
(d) acceleration clamp
        ↓
(e) deadband compensation
        ↓
(f) hard clamp
        ↓
(g) write PWM
        ↓
(h) publish state snapshot
```

The order matters.

### (a) Desired velocity

In position mode, the desired velocity is derived from:

```text
(target_us - position_us) / dt
```

and then constrained by the following stages.

### (b) Anti-windup

If the actuator is already at a physical limit and the requested motion continues farther in the same direction:

```text
position = max
requested velocity > 0
        ↓
desired velocity = 0
```

This prevents the controller from accumulating an implicit "command debt" while the actuator is already saturated.

### (c) Slew-rate limit

The requested velocity is limited by:

```c
SERVO_SLEW_MAX_US_PER_S
```

### (d) Acceleration clamp

The actual applied velocity is allowed to change only by:

```text
SERVO_ACCEL_MAX_US_PER_S2 × dt
```

This is the final dynamic protection against excessively abrupt actuator commands.

It is not a trajectory generator. The trajectory engine is responsible for motion planning; the actuator layer is responsible for ensuring that the resulting command remains physically constrained.

### (e) Deadband compensation

When the command changes direction, a calibrated `deadband_us` offset can be applied to compensate for mechanical backlash.

This is intentionally separate from trajectory reversal:

```text
Trajectory Engine
    → smooth reversal

Servo Actuator
    → backlash compensation
```

These are different physical problems and therefore belong to different layers.

### (f) Hard clamp

The final PWM command is constrained by calibrated minimum and maximum values.

This is the absolute physical boundary.

### (g) / (h)

The resulting PWM command is written to the servo output and a state snapshot is published through the project's seqlock mechanism for telemetry and UI readers.

---

## 8. Single Source of Truth for Servo Physical Limits

The actuator layer defines the physical servo limits:

```c
#define SERVO_SLEW_MAX_US_PER_S    5100.0f
#define SERVO_ACCEL_MAX_US_PER_S2  340000.0f
```

These values are derived from the selected servo characteristics and include practical margin for the real mechanical load.

The trajectory layer does not independently redefine the same actuator limits. Instead, the height-axis limits are derived from the actuator limits through the calibrated:

```text
µs/mm
```

conversion.

For example:

```c
#define TRAJ_HEIGHT_V_MAX \
    (SERVO_SLEW_MAX_US_PER_S / HEIGHT_K_US_PER_MM)

#define TRAJ_HEIGHT_A_MAX \
    (SERVO_ACCEL_MAX_US_PER_S2 / HEIGHT_K_US_PER_MM)
```

This avoids two independent copies of the same physical constraint drifting apart during tuning.

Changing the actuator-level physical constraint automatically propagates to the trajectory layer.

---

## 9. Why the Two Layers Use Different Margins

`TRAJ_ROLL_PITCH_V_MAX` uses the theoretical platform-angle velocity limit, while the actuator layer applies its own practical margin.

This is intentional.

The trajectory engine is a **soft motion-planning layer**.

The actuator layer is the **hard final boundary**.

Applying the same conservative margin twice would unnecessarily reduce responsiveness:

```text
Trajectory limit
      ↓
soft filtering
      ↓
Actuator limit with physical margin
      ↓
PWM
```

The final actuator layer is therefore the authoritative safety boundary.

---

## 10. Second-Order Inverse Kinematics

`control_ball_ik()` uses a second-order polynomial model for each servo:

```c
const float f[6] = {
    R,
    P,
    R2,
    P2,
    RP,
    1.0f
};
```

where:

```text
R  = Roll
P  = Pitch
R2 = Roll²
P2 = Pitch²
RP = Roll × Pitch
```

The coefficients are obtained through calibration using least-squares fitting.

This model captures nonlinear coupling between platform orientation and servo displacement more accurately than a purely linear mapping.

---

## 11. Height Offset and Total Clamp

`control_ball_apply_rph()` applies the height offset to all three servo axes before applying the final clamp:

```c
const float h_off =
    control_ball_height_offset_us(height_d);

s1 = clamp_f(
    s1 + h_off,
    -AXIS_OFFSET_MAX_US,
    AXIS_OFFSET_MAX_US);
```

The total command is clamped after the components are combined.

This is important.

Clamping each component separately and then adding them could still produce a total value outside the actual safe boundary:

```text
safe component A
+
safe component B
=
unsafe total
```

The final combined value must therefore be constrained after all relevant contributions have been applied.

---

## 12. Two Trajectory APIs for Two Different Motion Classes

`trajectory.h` intentionally keeps two compatible APIs.

| API | Usage | Motion assumption |
|---|---|---|
| `trajectory_replan()` / `trajectory_update()` | Balance mode | Continuous high-rate target updates; preserve current velocity/acceleration |
| `trajectory_start()` | Home / Calibration | Discrete point-to-point move starting from rest |

This is not unnecessary duplication.

The two operating modes represent different physical problems.

### Balance mode

There is no stable fixed endpoint. The target may change every control cycle.

Therefore:

```text
current trajectory state
        +
new target
        ↓
replan continuously
```

### Home / Calibration

The target is a discrete position and the movement begins from a known state.

Therefore:

```text
current position
        ↓
fixed target
        ↓
move to target
```

`trajectory_start_synced3()` is similarly appropriate for discrete three-axis moves where all three axes must arrive at a common endpoint at the same time.

That concept does not map directly to Balance mode because the target is continuously changing.

---

## 13. Jetson / Camera Loss Failsafe

When Ball mode is active and camera data is valid:

```c
if (s_ball_on && camera_ok) {
    /* use Roll_d / Pitch_d / Height_d from Jetson */
}
```

When Ball mode is disabled or Jetson/camera data becomes invalid:

```c
trajectory_replan(
    &s_traj_roll,
    s_roll_cur,
    s_traj_roll.v,
    s_traj_roll.a,
    0.0f,
    TRAJ_ROLL_PITCH_V_MAX,
    TRAJ_ROLL_PITCH_A_MAX);
```

The same trajectory engine is reused, with the target changed to:

```text
Roll  = 0
Pitch = 0
Height = 0
```

This returns the platform smoothly toward its neutral state instead of introducing a second emergency-motion implementation.

The behavior is deliberately **not an emergency E-stop**.

A temporary communication loss should not automatically produce a hard mechanical stop if a controlled return to the neutral state is still possible.

The actual emergency/safety path remains separate from this controlled fallback.

---

## 14. Why Reusing the Same Trajectory Engine Matters

Using the same trajectory implementation for both:

```text
Ball:ON
Ball:OFF / camera lost
```

has an important maintenance advantage.

A trajectory-engine bug fix or tuning change automatically affects both paths.

There are no two independent braking implementations that can gradually diverge.

The resulting architecture is:

```text
                Jetson target
                     │
                     ▼
              Trajectory Engine
                 /         \
                /           \
          Ball:ON        Ball:OFF
             │               │
             │          target = 0
             └───────┬───────┘
                     ▼
              Same motion limits
                     │
                     ▼
               Inverse Kinematics
                     │
                     ▼
               Servo Actuator
                     │
                     ▼
                    PWM
```

---

## 15. Engineering Summary

The actuator architecture separates **motion planning, nonlinear kinematic conversion, and final actuator protection**:

```text
Desired platform motion
        ↓
Acceleration-continuous trajectory
        ↓
Second-order inverse kinematics
        ↓
Servo-space slew / acceleration limits
        ↓
Deadband compensation
        ↓
Physical PWM clamp
        ↓
Servo
```

The separation is intentional:

- the trajectory engine reasons in physical platform coordinates;
- inverse kinematics converts platform motion into actuator commands;
- the actuator layer provides an independent final safety boundary.

This prevents the motion planner from becoming the only line of defense for physical actuator limits and allows Home, Calibration, and Balance modes to share the same low-level actuator protection.

The design also keeps the continuous Balance control path separate from discrete Home/Calibration motion semantics while reusing the same trajectory implementation for controlled failsafe return.

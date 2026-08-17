# 09 — Inverse Kinematics Calibration and Persistent Servo Model

> Based on the current implementation in `firmware-stm32h723/App/control/`, especially `calibration_data.c/h`, `control_ball_common.c/h`, the calibration mode, and the servo actuator interface. The calibration layer converts experimentally measured platform behavior into persistent coefficients used by the runtime inverse-kinematics path.

---

## 1. Calibration Is Treated as a Runtime Configuration Layer

The mechanical system does not rely on an ideal geometric model alone.

The real mapping between:

```text
Platform orientation
        ↓
Roll / Pitch
        ↓
S1 / S2 / S3 servo commands
```

contains practical effects from:

- servo neutral offsets;
- mechanical assembly tolerances;
- linkage geometry;
- nonlinear servo-to-platform behavior;
- cross-axis coupling;
- actuator deadband.

The firmware therefore separates calibration data from control logic.

```text
Calibration procedure
        ↓
calibration_data_t
        ↓
CRC + version + magic
        ↓
STM32 internal Flash
        ↓
runtime load
        ↓
control_ball_ik()
        ↓
servo_actuator
```

This allows the runtime controller to use measured system behavior without recompiling the control algorithm every time the mechanical calibration changes.

---

## 2. Why the Runtime IK Model Is Second Order

The runtime inverse-kinematics model uses six basis terms:

```c
R
P
R²
P²
R·P
1
```

For each servo:

```text
S1 = c10·R + c11·P
   + c12·R² + c13·P²
   + c14·R·P + c15

S2 = c20·R + c21·P
   + c22·R² + c23·P²
   + c24·R·P + c25

S3 = c30·R + c31·P
   + c32·R² + c33·P²
   + c34·R·P + c35
```

The implementation stores this as:

```c
ik_coef[3][6]
```

rather than a separate set of coefficients for each polynomial term.

The model is therefore a compact second-order approximation rather than a full analytical mechanical model.

---

## 3. Why a Linear Model Was Not Sufficient

A first-order model would be:

```text
S = a·Roll + b·Pitch + c
```

This assumes that the servo response changes approximately linearly across the operating region.

The measured mechanism has nonlinear behavior, so the runtime model adds:

```text
Roll²
Pitch²
Roll·Pitch
```

The cross term is particularly important because the effect of Roll can depend on the current Pitch, and vice versa.

Conceptually:

```text
Roll only
    → servo response

Pitch only
    → servo response

Roll + Pitch
    → not necessarily the simple sum
```

The `R·P` term gives the fitted model one way to represent this coupling without introducing a substantially more complex kinematic solver.

---

## 4. Calibration Data Structure

The calibration record contains both actuator calibration and IK calibration information.

The persistent record includes fields for:

```text
magic
version

S1_neutral
S1_min
S1_max

S2_neutral
S2_min
S2_max

S3_neutral
S3_min
S3_max

roll_offset
pitch_offset

ik_coef[3][6]

deadband_S1
deadband_S2
deadband_S3

CRC32
```

This is an intentional combination.

The controller needs both:

```text
Mechanical mapping
```

and:

```text
Actuator-specific limits / neutral points
```

to produce a usable command.

A mathematically correct IK result is not useful if the corresponding servo neutral point or physical range is wrong.

---

## 5. Calibration Versioning

The calibration record contains:

```c
magic
version
```

before the runtime accepts the stored data.

This matters because the project changed the calibration representation from the previous first-order model:

```text
A1, B1
A2, B2
A3, B3
```

to the current second-order representation:

```text
ik_coef[3][6]
```

The firmware deliberately does **not** attempt to reinterpret the old structure as the new one.

If the stored magic or version is incompatible:

```text
Flash data
    ↓
magic/version check fails
    ↓
defaults
    ↓
s_valid = false
    ↓
calibration must be performed again
```

This is safer than silently treating an old binary layout as a valid new calibration.

---

## 6. CRC Validation

The calibration record uses CRC32 for integrity verification.

The CRC is calculated over the calibration structure excluding its own `crc32` field:

```c
uint32_t len_words =
    (sizeof(*d) - sizeof(d->crc32))
    / sizeof(uint32_t);

return HAL_CRC_Calculate(
    &hcrc,
    (uint32_t *)d,
    len_words);
```

The load sequence is:

```text
Read Flash
    ↓
Check magic
    ↓
Check version
    ↓
Calculate CRC
    ↓
Compare with stored CRC
    ↓
valid / invalid
```

If CRC validation fails, the firmware does not expose the corrupted record as active calibration.

Instead it returns to the default invalid state.

This protects against using partially corrupted calibration parameters as if they were valid control coefficients.

---

## 7. Default Calibration Is Designed to Fail Safe

`calibration_data_set_defaults()` clears the structure first:

```c
memset(out, 0, sizeof(*out));
```

The second-order IK coefficients therefore start at zero.

The resulting default IK output is:

```text
S1 = 0
S2 = 0
S3 = 0
```

rather than a random or partially initialized polynomial.

The calibration state is also marked invalid:

```c
s_valid = false;
```

This is an important distinction:

```text
No valid calibration
        ≠
Valid calibration with zero motion
```

The firmware knows whether the stored calibration has passed validation.

---

## 8. Neutral Servo Positions Are Empirical

The default calibration contains experimentally measured neutral values:

```text
S1_neutral = 1586 µs
S2_neutral = 1496 µs
S3_neutral = 1564 µs
```

These values are not assumed to be identical.

That reflects the physical reality of a three-servo mechanism:

```text
Servo 1 ≠ Servo 2 ≠ Servo 3
```

even when all three are intended to produce the same mechanical platform state.

The neutral values therefore belong in calibration data rather than being hard-coded as a single theoretical center point.

---

## 9. Physical Servo Limits Are Derived from Neutral

When calibration data is saved, the firmware does not trust arbitrary min/max values supplied by the caller.

Instead:

```text
S1_min = S1_neutral - CALIB_TILT_MAX_US
S1_max = S1_neutral + CALIB_TILT_MAX_US

S2_min = S2_neutral - CALIB_TILT_MAX_US
S2_max = S2_neutral + CALIB_TILT_MAX_US

S3_min = S3_neutral - CALIB_TILT_MAX_US
S3_max = S3_neutral + CALIB_TILT_MAX_US
```

This creates a consistent relationship:

```text
calibrated neutral
        +
allowed calibrated travel
        ↓
physical command window
```

The important design choice is that the saved limits are recomputed during the persistence step.

The calibration caller cannot accidentally enlarge the stored range by passing an unsafe min/max value.

---

## 10. Servo Deadband Is Calibrated Independently

The record stores:

```text
deadband_S1
deadband_S2
deadband_S3
```

rather than one global deadband.

This accounts for differences between the three mechanical channels.

Deadband compensation is applied by the actuator layer, not by the IK polynomial.

That separation keeps two different effects independent:

```text
IK model
    → maps platform orientation to desired servo position

Deadband compensation
    → compensates local actuator backlash / static response
```

This is important for tuning because changing deadband should not require refitting the platform-level polynomial.

---

## 11. Calibration Data Persistence in STM32H723 Flash

The calibration record is stored in a dedicated Flash region:

```text
CALIB_FLASH_ADDR = 0x080E0000
CALIB_REGION_SIZE = 128 KB
```

The linker configuration reserves this area so application code does not overlap the calibration storage.

The persistence flow is:

```text
Calibration result
        ↓
copy to temporary record
        ↓
set magic/version
        ↓
recalculate limits
        ↓
calculate CRC
        ↓
Flash unlock
        ↓
sector erase
        ↓
Flash-word programming
        ↓
Flash lock
        ↓
read-back verification
        ↓
activate calibration
```

The runtime therefore does not simply assume that a successful Flash API call means the complete record was persisted correctly.

---

## 12. Read-Back Verification

After programming, the firmware reads the record back from Flash:

```c
memcpy(
    &verify,
    (const void *)CALIB_FLASH_ADDR,
    sizeof(verify));
```

and compares it against the intended record:

```c
memcmp(&verify, &tmp, sizeof(tmp))
```

If the comparison fails:

```text
s_valid = false
return false
```

This provides a second integrity check after programming.

The verification sequence is therefore:

```text
Software calculated record
        ↓
Flash programming
        ↓
Physical Flash read-back
        ↓
byte-for-byte comparison
```

---

## 13. IWDG Handling During Flash Erase

Flash sector erase is a blocking operation.

The project documents that the normal IWDG configuration is approximately:

```text
Prescaler = 32
Reload    = 500
```

while erasing the 128 KB calibration sector can take significantly longer than the normal watchdog window.

The calibration code therefore temporarily widens the watchdog timeout:

```text
Normal IWDG
     ↓
widen timeout
     ↓
Flash erase + program
     ↓
restore original timeout
```

The widened configuration is:

```text
Prescaler = 256
Reload    = 4000
```

The original configuration is restored immediately after the Flash operation.

This is an important example of distinguishing:

```text
expected long blocking hardware operation
```

from:

```text
unexpected firmware stall
```

The watchdog is not permanently disabled; its normal supervision window is restored after the calibration transaction.

---

## 14. Flash Alignment and STM32H7 Programming

The temporary Flash buffer is rounded to a 32-byte boundary:

```c
uint8_t buf[
    ((sizeof(tmp) + 31u) / 32u) * 32u
];
```

and initialized to:

```c
0xFF
```

before the calibration structure is copied into it.

Programming then proceeds in 32-byte Flash-word increments.

This matches the STM32H7 Flash programming granularity used by the implementation and prevents the final partial record from creating an invalid programming transaction.

The padding bytes remain deterministic:

```text
calibration record
+
0xFF padding
```

---

## 15. Runtime Application of Calibration

After valid calibration data is loaded, the actuator calibration is explicitly applied:

```c
servo_actuator_set_calib(
    SERVO_CH_S1,
    S1_neutral,
    S1_min,
    S1_max,
    deadband_S1);
```

and similarly for S2 and S3.

The runtime relationship is therefore:

```text
Persistent calibration
        ↓
Servo neutral
Servo limits
Deadband
        ↓
Servo Actuator
```

while the IK coefficients remain part of the platform-level mapping:

```text
Roll / Pitch
        ↓
ik_coef[3][6]
        ↓
S1 / S2 / S3
```

This keeps actuator calibration and platform calibration logically connected but not collapsed into one computation.

---

## 16. Calibration and Runtime Control Are Separate Responsibilities

The calibration mode is responsible for obtaining and storing experimentally derived parameters.

The runtime balance controller is responsible for using those parameters.

Conceptually:

```text
CALIB mode
   │
   ├── collect / determine calibration
   ├── persist calibration
   └── mark data valid
            │
            ▼
      persistent Flash
            │
            ▼
        boot / load
            │
            ▼
      runtime control
            │
            ├── IK
            └── servo actuator
```

This avoids putting Flash operations into the high-rate control path.

The balance controller should use already validated calibration data rather than performing erase/program operations while closed-loop control is active.

---

## 17. Calibration Data Is Not the Same as Controller Tuning

The calibration layer should be distinguished from PID or trajectory tuning.

| Parameter class | Purpose |
|---|---|
| `ik_coef[3][6]` | Platform orientation → servo command mapping |
| `S*_neutral` | Mechanical neutral point |
| `S*_min/max` | Servo physical command range |
| `deadband_S*` | Local actuator backlash / static-response compensation |
| `roll_offset` / `pitch_offset` | Platform/sensor reference offset |
| PID gains | Closed-loop response |
| trajectory limits | Motion profile constraints |

This separation is useful during system tuning.

If the platform moves in the wrong direction for a given Roll/Pitch command, the first question is not necessarily "change PID gains".

The mapping itself may be wrong.

Likewise, if the platform reaches the correct orientation but oscillates, the issue may belong to the controller rather than the IK calibration.

---

## 18. Calibration as a Measured System Identification Step

The calibration process can be viewed as a small system-identification problem:

```text
Known platform orientation
        ↓
Measured servo response
        ↓
Feature vector
[R, P, R², P², R·P, 1]
        ↓
Least-squares fit
        ↓
ik_coef[3][6]
        ↓
Runtime polynomial
```

The fitted coefficients therefore represent the behavior of the assembled mechanical system rather than an ideal CAD-only model.

This is particularly appropriate for a small three-actuator platform where mechanical tolerances and servo mounting geometry can materially affect the mapping.

---

## 19. Why the Polynomial Is Kept Compact

A more complex model could use:

- higher-order polynomial terms;
- lookup tables;
- piecewise interpolation;
- a full geometric inverse-kinematics solution.

The current implementation deliberately uses:

```text
6 coefficients × 3 servos
```

This has practical embedded-system advantages:

```text
small persistent dataset
+
small runtime computation
+
no iterative solver
+
deterministic execution
```

For a real-time STM32 control path, the polynomial evaluation is straightforward and bounded.

The calibration data also fits comfortably inside the reserved Flash region.

---

## 20. Calibration Data Lifecycle

The complete lifecycle is:

```text
Mechanical setup
       ↓
Calibration procedure
       ↓
Measured data
       ↓
Second-order coefficient fitting
       ↓
calibration_data_t
       ↓
magic + version + CRC
       ↓
Flash erase/program
       ↓
read-back verification
       ↓
boot-time validation
       ↓
apply servo calibration
       ↓
runtime IK
       ↓
servo actuator limits
```

At boot:

```text
Flash record valid?
      │
   ┌──┴──┐
  yes    no
   │      │
   ▼      ▼
load    defaults
   │      │
   ▼      ▼
apply   invalid
calib   state
```

The controller therefore does not silently treat corrupted or obsolete calibration data as valid.

---

## 21. Engineering Trade-Offs

The current design makes several deliberate trade-offs.

### Second-order polynomial instead of full analytical IK

**Advantage**

- compact;
- deterministic;
- easy to evaluate at runtime;
- captures nonlinear and cross-axis behavior.

**Trade-off**

- valid only within the calibrated operating region;
- requires representative calibration data;
- does not automatically model changes in mechanical geometry.

### Internal Flash instead of external EEPROM

**Advantage**

- no additional bus/peripheral;
- calibration is stored with the MCU;
- simple boot-time loading.

**Trade-off**

- sector erase is relatively coarse;
- Flash write/erase is blocking;
- write endurance must be considered if calibration is saved frequently.

The current implementation addresses the blocking operation with temporary watchdog widening and restores normal supervision afterward.

---

## 22. What the Calibration Layer Guarantees — and What It Does Not

The calibration layer provides:

```text
✓ versioned calibration data
✓ CRC validation
✓ safe default state
✓ experimentally derived neutral positions
✓ second-order IK coefficients
✓ per-servo deadband
✓ derived servo limits
✓ Flash read-back verification
✓ explicit validity state
```

It does not by itself guarantee:

```text
✗ closed-loop stability
✗ perfect mechanical accuracy
✗ optimal PID gains
✗ validity outside the calibrated Roll/Pitch range
✗ immunity to mechanical changes after calibration
```

Those remain responsibilities of the control, mechanical, and actuator layers.

This separation is important when evaluating the system experimentally.

---

## 23. Engineering Summary

The calibration architecture can be summarized as:

```text
             REAL MECHANISM
                   │
                   ▼
        Experimental calibration
                   │
                   ▼
       Second-order system model
                   │
        ┌──────────┴──────────┐
        │                     │
   IK coefficients       Servo calibration
        │                     │
        │              neutral / limits /
        │                 deadband
        └──────────┬──────────┘
                   ▼
             Persistent Flash
                   │
            magic/version/CRC
                   │
                   ▼
             Boot-time validation
                   │
                   ▼
             Runtime controller
                   │
                   ▼
          Roll / Pitch → S1/S2/S3
                   │
                   ▼
            Servo Actuator
                   │
                   ▼
                  PWM
```

The key architectural decision is to treat calibration as **validated persistent system data**, not as constants scattered through the control code.

This gives the runtime controller a deterministic, compact model of the actual assembled mechanism while keeping mechanical calibration, actuator limits, controller tuning, and safety constraints as separate engineering concerns.

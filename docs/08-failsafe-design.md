# 08 — Failsafe Design and Fault Containment

> Based on the current implementation in `firmware-stm32h723/App/tasks/`, `App/comm/`, `App/control/`, `App/bsp/`, and the Jetson CAN/control path. The design is organized around fault detection, fault propagation, controlled fallback, and an independent hardware reset mechanism.

---

## 1. Failsafe Philosophy

The system is a distributed physical control system:

```text
Camera / Vision
      │
      ▼
Jetson Orin Nano
      │
      │ CAN
      ▼
STM32H723
      │
      ├── IMU
      ├── Control Loop
      ├── Trajectory
      └── Servo Actuators
```

A communication or software failure must therefore not be treated as a simple application error.

A stale ball position, a stalled control task, or a lost CAN link can eventually become a physical actuator problem.

The failsafe architecture separates four responsibilities:

```text
1. Detect
      ↓
2. Propagate fault
      ↓
3. Move to a controlled safe behavior
      ↓
4. Reset the MCU if critical execution health is lost
```

This separation is important because no single mechanism can cover all failure classes.

For example:

- CAN heartbeat detects communication liveness.
- Application-data timeouts detect stale control data.
- The state machine provides an explicit fault state.
- The trajectory engine provides a controlled return toward neutral.
- Servo actuator limits constrain the final PWM command.
- The IWDG provides recovery if critical firmware execution stops.

---

## 2. Fault Sources Considered by the Architecture

The current implementation contains protection paths for several practical failure modes.

| Fault source | Detection / containment |
|---|---|
| STM32 critical task stops executing | IWDG alive-bit supervision |
| Jetson CAN heartbeat disappears | CAN heartbeat timeout |
| Jetson application data becomes stale | Application-data liveness timeout |
| FDCAN enters Bus-Off | Application-level FDCAN restart |
| Ball/camera data becomes invalid | Ball-control fallback / neutral trajectory |
| SPI/DMA transaction fails | SPI error callback and diagnostic counters |
| IMU interrupt/DMA path stops delivering samples | Bounded task wait + watchdog supervision |
| Servo command exceeds calibrated range | Final PWM hard clamp |
| Excessive servo command rate | Slew-rate and acceleration limits |
| Target reverses while platform is moving | Explicit trajectory braking |
| TFT initialization races with control startup | Explicit TFT-ready / boot synchronization |

The important point is that these protections operate at different layers.

A fault in one subsystem should not require every other subsystem to implement the same detection mechanism.

---

## 3. Critical-Task Watchdog

The STM32 watchdog supervises the tasks that are required for continued safe control:

```c
#define ALIVE_BIT_CONTROL_LOOP   (1u << 0)
#define ALIVE_BIT_IMU_FUSION     (1u << 1)
#define ALIVE_BIT_CAN_RX         (1u << 2)

#define ALIVE_MASK_EXPECTED \
    (ALIVE_BIT_CONTROL_LOOP | \
     ALIVE_BIT_IMU_FUSION | \
     ALIVE_BIT_CAN_RX)
```

Each critical task reports that it is alive during normal execution.

`WatchdogTask` runs at a lower rate and atomically snapshots the alive mask.

Conceptually:

```text
ControlLoop ──────┐
IMU Fusion ───────┼──► alive mask
CAN RX ───────────┘
                       │
                       ▼
                WatchdogTask
                       │
             all required bits?
                  │          │
                 yes         no
                  │          │
             refresh IWDG    do not refresh
                                │
                                ▼
                             MCU reset
```

The watchdog is therefore not simply a periodic "kick".

A task must actively demonstrate progress before the hardware watchdog is refreshed.

---

## 4. Why the Watchdog Does Not Supervise Every Task

The expected alive mask contains only:

```text
ControlLoop
IMU Fusion
CAN RX
```

Tasks such as:

- Display;
- Button UI;
- CAN TX;

do not contribute to the critical mask.

This is deliberate.

A display driver failure should not prevent the controller from maintaining its core safety path.

Conversely, if the control loop or IMU fusion task stops progressing, continuing to run the system indefinitely would be unsafe.

The watchdog therefore represents **functional criticality**, not simply "all tasks must execute".

---

## 5. Watchdog Recovery Is Different from Application Failsafe

There are two fundamentally different recovery classes.

### Application-level fault

The firmware is still executing, but an external dependency is unavailable.

Example:

```text
Jetson stops sending ball data
        ↓
STM32 still runs normally
        ↓
detect stale data
        ↓
enter controlled fallback
```

The MCU does not need to reset.

### Execution-level fault

A critical task stops executing because of a deadlock, stack failure, severe blocking condition, or other firmware fault.

Example:

```text
ControlLoopTask stops
        ↓
no alive bit
        ↓
WatchdogTask refuses IWDG refresh
        ↓
hardware reset
```

The distinction prevents unnecessary resets during normal communication failures while still providing an independent recovery path for firmware execution failures.

---

## 6. CAN Heartbeat Supervision

The CAN protocol defines a dedicated heartbeat path.

The STM32 and Jetson exchange heartbeat frames independently from the normal application data.

The STM32 uses:

```c
#define CAN_HEARTBEAT_TIMEOUT_MS 200u
```

The heartbeat mechanism answers:

> "Is the remote processor still participating in the communication protocol?"

If the expected heartbeat is not received within the timeout:

```text
Heartbeat timeout
       ↓
fault condition
       ↓
state-machine fault event
       ↓
ERROR / safe handling path
```

The heartbeat is intentionally separate from the normal ball-position messages.

A system can continue receiving some data while the intended control/health relationship has already become invalid.

---

## 7. Application-Data Liveness

Heartbeat alone is not sufficient.

The controller also observes whether the actual application data required for balancing remains fresh.

For the Jetson-to-STM32 path, the relevant data includes:

```text
BALL_POS
BALL_VEL
BALL_STATE
ATTITUDE_DESIRED
```

The control decision is therefore conceptually:

```text
heartbeat healthy?
       +
application data recent?
       +
ball state valid?
       │
       ▼
allow Jetson-derived balance target
```

If the ball/camera data becomes stale, the STM32 must not continue indefinitely using the last received target.

This creates a second fault boundary:

```text
CAN link alive
      ≠
control data valid
```

That distinction is important in a distributed control system.

---

## 8. Controlled Fallback When Ball Data Is Lost

The Ball mode does not treat loss of camera data as permission to hold the last target forever.

When Ball mode becomes invalid, the control path can replan toward the neutral platform state:

```text
Roll  → 0
Pitch → 0
Height → 0
```

The same trajectory engine used for normal motion is reused for this fallback.

Conceptually:

```text
Normal balance target
        │
        │ camera / Jetson becomes invalid
        ▼
Target validity check
        │
        ▼
Neutral target
(Roll=0, Pitch=0, Height=0)
        │
        ▼
Trajectory replan
        │
        ▼
IK
        │
        ▼
Servo actuator limits
        │
        ▼
PWM
```

This is a **controlled fallback**, not an emergency E-stop.

The objective is to remove stale external commands while avoiding an unnecessary discontinuous actuator command.

---

## 9. Why the Fallback Reuses the Trajectory Engine

A separate "emergency movement" implementation would introduce another motion-control path with its own:

- acceleration behavior;
- velocity limits;
- boundary handling;
- tuning parameters.

The current architecture instead reuses the existing trajectory implementation.

This means:

```text
Normal target
      │
      ▼
same trajectory constraints
      │
      ├── normal operation
      │
      └── neutral fallback
```

The design reduces the number of independent actuator-control paths that must be validated.

The Servo Actuator remains the final protection layer regardless of which control mode generated the target.

---

## 10. Final Actuator Safety Boundary

The actuator layer applies physical constraints after trajectory generation and inverse kinematics.

The processing path is:

```text
Desired motion
      ↓
Trajectory
      ↓
Inverse Kinematics
      ↓
Desired servo position
      ↓
Anti-windup
      ↓
Slew-rate limit
      ↓
Acceleration limit
      ↓
Deadband compensation
      ↓
Hard PWM clamp
      ↓
Servo output
```

The hard clamp is important because the trajectory planner is not the only caller of the actuator interface.

Even if an upstream calculation produces an unexpected value, the final actuator layer still constrains the command to the calibrated servo range.

This creates a defense-in-depth boundary:

```text
Planning constraint
        +
IK output constraint
        +
Actuator dynamic constraint
        +
Final PWM clamp
```

---

## 11. Servo Dynamic Protection During Fault Recovery

A communication fault should not bypass the actuator dynamic limits.

For example, when the target changes from:

```text
Roll = +5°
```

to:

```text
Roll = 0°
```

the trajectory engine determines the controlled motion profile.

The servo actuator independently enforces:

```text
maximum velocity
maximum acceleration
physical PWM range
```

Therefore:

```text
CAN fault
   ↓
neutral target
   ↓
trajectory
   ↓
IK
   ↓
servo limits
```

rather than:

```text
CAN fault
   ↓
direct PWM jump
```

This separation is particularly important because the system drives physical servos rather than purely simulated state.

---

## 12. Trajectory Braking on Target Reversal

A target reversal is itself a potential safety event.

If the platform is moving in one direction and the new target is on the opposite side, the trajectory engine does not instantly reverse acceleration.

It inserts a braking phase:

```text
Current velocity
       ↓
BRAKE
       ↓
velocity = 0
       ↓
ACCEL
       ↓
CRUISE
       ↓
DECEL
```

This prevents a target update from becoming an abrupt actuator reversal.

The same mechanism applies whether the new target comes from normal control updates or from a controlled return toward the neutral position.

---

## 13. IMU Acquisition Fault Containment

The IMU path is protected independently of CAN.

The MPU6500 uses:

```text
data-ready EXTI
      ↓
SPI2 DMA
      ↓
DMA completion callback
      ↓
FreeRTOS task notification
      ↓
Kalman fusion
```

The task notification wait is bounded rather than infinite.

This prevents the fusion task from becoming permanently blocked if the expected sample event stops arriving.

The system also records SPI/DMA diagnostic counters, including:

```text
DMA start attempts
busy skips
DMA start failures
SPI error count
last SPI error code
```

This makes the sensor failure observable during integration.

The watchdog then provides the independent execution-level recovery path if the critical task itself stops making progress.

---

## 14. FDCAN Bus-Off Recovery

The STM32 monitors the FDCAN protocol status.

When Bus-Off is detected, the application performs a controlled restart:

```text
FDCAN Bus-Off
      ↓
HAL_FDCAN_Stop()
      ↓
HAL_FDCAN_Start()
      ↓
resume CAN communication
```

This handles a communication-layer failure without requiring an MCU reset.

It is therefore another example of layered recovery:

```text
Recover locally first
        ↓
reset only when necessary
```

A transient CAN fault should not automatically reboot the complete controller if the FDCAN peripheral can recover safely.

---

## 15. State Machine as the Fault Propagation Layer

Fault detection and fault handling are intentionally separated.

For example:

```text
CAN RX
  │
  │ detects heartbeat timeout
  ▼
fault/event
  │
  ▼
State Machine
  │
  ▼
ERROR / safe behavior
```

This avoids embedding complete system-state transitions inside every peripheral driver.

The CAN driver should detect communication health.

The watchdog should supervise task progress.

The state machine owns the system-level mode transition.

This keeps fault handling centralized and makes the behavior easier to reason about.

---

## 16. Boot-Time Safety and Initialization Ordering

The display subsystem also participates in the system's startup dependency chain.

The control task must not assume that the TFT subsystem is ready simply because the scheduler has started.

The implementation uses explicit events:

```c
EVT_BIT_TFT_READY
EVT_BIT_BOOT_DONE
```

The sequence is:

```text
DisplayTask
    │
    ├── TFT initialization
    ├── Boot screen initialization
    └── set TFT_READY
             │
             ▼
ControlLoopTask
    │
    └── allowed to use Boot UI
             │
             ▼
       BOOT_DONE
             │
             ▼
DisplayTask leaves Boot screen
```

This prevents a startup race from becoming a permanent deadlock.

The wait is also bounded so that the control task continues satisfying watchdog supervision while waiting for initialization dependencies.

This is part of the failsafe design because startup sequencing is itself a safety-critical dependency.

---

## 17. Fault Handling Hierarchy

The resulting hierarchy is:

```text
                    SYSTEM HEALTH
                         │
          ┌──────────────┼──────────────┐
          │              │              │
      Hardware       Execution      Application
      protection      health          health
          │              │              │
          │              │              ├── CAN heartbeat
          │              │              ├── data freshness
          │              │              └── ball validity
          │              │
          │              └── critical-task alive mask
          │
          ├── servo hard clamp
          ├── slew / acceleration limits
          └── peripheral error handling
                         │
                         ▼
                  State Machine
                         │
          ┌──────────────┴──────────────┐
          │                             │
    Controlled fallback             IWDG reset
          │                             │
          ▼                             ▼
    Neutral trajectory             MCU restart
```

The architecture therefore does not rely on one "failsafe function".

Instead, different failure classes are contained at the layer where they can be detected most reliably.

---

## 18. Example Fault Scenarios

### Scenario A — Jetson stops sending CAN

```text
Heartbeat timeout
       ↓
CAN fault event
       ↓
state-machine fault handling
       ↓
stop using stale external target
       ↓
controlled neutral trajectory
       ↓
servo safety limits remain active
```

The STM32 remains operational.

### Scenario B — Camera pipeline stops updating

```text
No fresh BALL_POS / BALL_VEL
       ↓
application-data timeout
       ↓
Ball target invalid
       ↓
neutral target
       ↓
trajectory-controlled return
```

The last camera position is not treated as permanently valid.

### Scenario C — ControlLoopTask deadlocks

```text
ControlLoopTask stops
       ↓
alive bit missing
       ↓
WatchdogTask does not refresh IWDG
       ↓
hardware watchdog expires
       ↓
MCU reset
```

This protects against a firmware execution failure that cannot be handled by the normal state machine.

### Scenario D — CAN peripheral enters Bus-Off

```text
FDCAN Bus-Off
       ↓
HAL_FDCAN_Stop()
       ↓
HAL_FDCAN_Start()
       ↓
CAN communication recovery
```

A peripheral-level fault is recovered locally without immediately rebooting the controller.

### Scenario E — Servo command exceeds a physical boundary

```text
Unexpected IK / target value
       ↓
servo actuator hard clamp
       ↓
safe PWM range
```

The actuator layer remains the final command boundary.

---

## 19. Why the Design Uses Multiple Independent Mechanisms

The different mechanisms intentionally cover different fault assumptions:

| Mechanism | What it proves |
|---|---|
| CAN heartbeat | Remote processor is communicating |
| Application-data timeout | Required control data is fresh |
| Ball-state validation | Vision input is usable |
| FDCAN Bus-Off recovery | CAN peripheral can recover locally |
| IMU bounded wait | Sensor task does not block indefinitely |
| Critical-task alive mask | Safety-critical tasks are executing |
| IWDG | MCU can recover from severe execution failure |
| Trajectory fallback | Stale external target is replaced safely |
| Servo dynamic limits | Motion remains bounded |
| Servo hard clamp | Final PWM remains within physical limits |
| State machine | Faults become explicit system states |

No single mechanism is expected to detect every failure.

This is the core design principle of the failsafe architecture:

> **Detect faults close to their source, propagate system-level consequences through the state machine, and keep an independent hardware recovery path for failures that prevent normal software recovery.**

---

## 20. Engineering Summary

The failsafe architecture can be summarized as:

```text
                 Fault
                   │
          ┌────────┴────────┐
          │                 │
      Detect locally     Detect globally
          │                 │
          └────────┬────────┘
                   ▼
             Fault event
                   │
                   ▼
             State Machine
                   │
          ┌────────┴────────┐
          │                 │
   Controlled fallback   Critical execution
          │                 │
          ▼                 ▼
 Neutral trajectory       IWDG reset
          │
          ▼
     Inverse Kinematics
          │
          ▼
   Servo safety limits
          │
          ▼
        PWM
```

The important architectural property is that **communication loss, stale sensor data, peripheral faults, software execution failures, and actuator-limit violations do not all share the same recovery mechanism**.

Each failure is handled at the lowest practical layer, while the state machine coordinates system-level behavior and the independent IWDG provides a final recovery mechanism when normal software execution can no longer be trusted.

This makes the control system more predictable under abnormal conditions without turning every transient fault into an immediate MCU reset.

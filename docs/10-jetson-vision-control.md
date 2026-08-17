# 10 — Jetson Vision and Control Pipeline

> Based on the current `jetson-vision-control` implementation and its public source tree. The Jetson side is organized as a Linux/C++ application with separate camera capture, ball detection, control, CAN RX/TX, system-state, watchdog, and video-recording components. The repository exposes these boundaries explicitly through `src/`, `include/`, `tests/`, `tools/`, and `calib/`.

---

## 1. Role of the Jetson Subsystem

The Jetson Orin Nano is responsible for the computationally heavier part of the control system:

```text
Camera
   │
   ▼
Camera Capture
   │
   ▼
Ball Detection
   │
   ▼
Ball State
(position / velocity / validity)
   │
   ▼
Control Loop
   │
   ├── PID
   │
   ▼
Desired platform motion
   │
   ▼
CAN TX
   │
   ▼
STM32H723
```

The STM32 remains responsible for the real-time actuator side.

This creates a deliberate split:

```text
Jetson
    vision
    estimation
    high-level control
    CAN communication

STM32
    IMU acquisition
    real-time control execution
    trajectory generation
    inverse kinematics
    servo constraints
    actuator output
    watchdog / failsafe
```

The repository description identifies the project as an `STM32H723 + Jetson Orin Nano` ball-balancing system using FreeRTOS, CAN, computer vision, and PID control. citeturn0view0

---

## 2. Jetson Software Architecture

The Jetson application is separated into explicit source-level responsibilities:

```text
jetson-vision-control/
│
├── calib/
├── include/
│   ├── ball_detector.hpp
│   ├── camera_pipeline.hpp
│   ├── can_id.h
│   ├── can_protocol.h
│   ├── can_transport.hpp
│   ├── operating_mode.hpp
│   ├── pid_controller.hpp
│   ├── seqlock.hpp
│   ├── system_state.hpp
│   └── task_*.hpp
│
├── src/
│   ├── camera_pipeline.cpp
│   ├── can_transport.cpp
│   ├── main.cpp
│   ├── system_state.cpp
│   ├── task_ball_detect.cpp
│   ├── task_camera_capture.cpp
│   ├── task_can_rx.cpp
│   ├── task_can_tx.cpp
│   ├── task_control_loop.cpp
│   ├── task_video_record.cpp
│   └── task_watchdog.cpp
│
├── tests/
├── tools/
└── CMakeLists.txt
```

This is an important architectural property.

The vision pipeline is not implemented as one large `main()` loop. Camera acquisition, detection, control, CAN transport, recording, and health supervision have explicit interfaces.

The public repository exposes all of these source boundaries. citeturn1view0turn1view1turn1view2

---

## 3. Camera Capture Is Separated from Ball Detection

The Jetson side has two separate task interfaces:

```text
task_camera_capture
task_ball_detect
```

and a dedicated:

```text
camera_pipeline
```

module.

The intended data flow is therefore:

```text
Camera device
     │
     ▼
CameraPipeline
     │
     ▼
Camera frame state
     │
     ▼
BallDetectTask
     │
     ▼
Ball state
```

This separation avoids coupling camera-device handling to the computer-vision algorithm.

The camera task can deal with:

- frame acquisition;
- camera lifecycle;
- frame availability;

while the ball detector can focus on:

- processing a frame;
- extracting ball information;
- validating the result.

That makes the vision algorithm replaceable without rewriting the camera management layer.

---

## 4. Ball Detection Is an Independent Processing Stage

The Jetson repository exposes:

```text
ball_detector.hpp
task_ball_detect.hpp
task_ball_detect.cpp
```

as separate interfaces.

This creates a clean boundary:

```text
Raw image
   ↓
Ball detector
   ↓
Detected ball state
```

The control loop should not need to know how the ball was detected.

That distinction is important for a robotics system because the detector can evolve independently:

```text
HSV / contour based detection
        ↓
different detector
        ↓
learned detector
```

without changing the downstream control interface, provided the same ball-state contract is maintained.

---

## 5. Vision Output Should Be Treated as a Measurement

The detected ball position is not equivalent to a commanded actuator state.

Conceptually:

```text
Image
  ↓
Measurement
  ↓
Validation
  ↓
Ball state
  ↓
Control
```

The control layer therefore consumes a state representation rather than directly manipulating camera pixels.

This is an important separation between:

```text
Perception
```

and:

```text
Control
```

A vision failure should invalidate the measurement rather than silently becoming a zero or arbitrary control command.

The system-level fallback behavior is handled downstream by the control/state-management layers.

---

## 6. Ball State and Shared-State Synchronization

The Jetson source tree contains a dedicated:

```text
seqlock.hpp
system_state.hpp
system_state.cpp
```

layer.

This is consistent with the application's need to share high-rate state between independent processing tasks without turning every read into a blocking mutex operation.

A conceptual data path is:

```text
Camera task
      │
      ▼
frame state

Ball detection task
      │
      ▼
ball state

Control task
      │
      ▼
control output
```

The shared-state abstraction keeps the synchronization policy out of the individual processing algorithms.

This is especially useful for the control loop, where the preferred behavior is:

```text
read latest valid state
```

rather than:

```text
wait for another task to release a mutex
```

---

## 7. Control Loop Is Independent from CAN Transport

The repository contains separate modules:

```text
task_control_loop
task_can_tx
task_can_rx
can_transport
```

This is an important architectural separation.

The control algorithm should generate the desired state independently from the details of SocketCAN transport.

```text
Control Loop
     │
     ▼
Desired control state
     │
     ▼
CAN TX task
     │
     ▼
CAN transport
     │
     ▼
SocketCAN
```

Likewise, incoming STM32 state is received through the CAN path and made available to the Jetson application rather than being parsed directly inside the PID implementation.

This keeps the control algorithm transport-agnostic.

---

## 8. PID Controller as a Dedicated Component

The public include tree contains:

```text
pid_controller.hpp
```

rather than embedding the PID calculation directly in the task implementation.

This gives the control loop a clear responsibility:

```text
read current ball state
        +
read target state
        ↓
PID controller
        ↓
desired control output
```

while the PID class/module owns:

```text
error calculation
integral state
derivative behavior
gain configuration
output calculation
```

This separation is useful for testing because the controller can be exercised independently from:

- camera hardware;
- CAN hardware;
- task scheduling.

---

## 9. Control Loop and Real-Time Scheduling

The Jetson side is a Linux application rather than a microcontroller firmware scheduler.

The architecture therefore separates:

```text
high-rate application work
```

from:

```text
Linux device I/O
```

through task-level components.

The repository contains dedicated tasks for:

```text
camera capture
ball detection
control loop
CAN RX
CAN TX
video recording
watchdog
```

This allows expensive or non-control work such as video recording to remain outside the core control calculation.

The design objective is not to make every task equally real-time.

Instead:

```text
Control path
    ↑
higher timing importance

Telemetry / recording
    ↑
lower timing importance
```

This is an appropriate separation for a Linux-based robotics controller.

---

## 10. Video Recording Is Kept Out of the Core Control Path

The Jetson source tree contains:

```text
task_video_record
```

as a separate task.

This is important because video encoding or disk I/O can introduce:

```text
CPU load
memory traffic
filesystem latency
```

that should not directly determine the timing of the control calculation.

The intended architecture is therefore:

```text
Camera
  ├──────────────► Ball detection ──► Control
  │
  └──────────────► Video recording
```

rather than:

```text
Camera
  ↓
record video
  ↓
process frame
  ↓
control
```

This separation also makes debugging easier because recorded data can be inspected without changing the fundamental control path.

---

## 11. CAN Protocol Is Shared with the STM32 Architecture

The Jetson side contains:

```text
can_id.h
can_protocol.h
can_transport.hpp
```

This mirrors the STM32 CAN protocol layer.

The communication boundary is therefore explicit:

```text
Jetson application state
        ↓
protocol encoding
        ↓
CAN transport
        ↓
CAN bus
        ↓
STM32 protocol decoder
        ↓
STM32 control state
```

The protocol definition should remain independent from the SocketCAN implementation.

This allows:

```text
can_protocol
```

to define:

- message identifiers;
- payload representation;
- units;
- field encoding;

while:

```text
can_transport
```

handles:

- SocketCAN;
- socket configuration;
- frame transmission/reception;
- Linux-specific transport behavior.

This separation is particularly useful when testing protocol logic without requiring the physical CAN interface.

---

## 12. CAN RX and TX Are Separate Tasks

The Jetson implementation explicitly contains:

```text
task_can_rx
task_can_tx
```

rather than one combined CAN task.

The separation is useful because receive processing and transmit scheduling have different responsibilities.

Conceptually:

```text
CAN RX
  │
  ▼
STM32 state / feedback
  │
  ▼
System state

Control loop
  │
  ▼
desired state
  │
  ▼
CAN TX
```

The control loop therefore does not need to block while waiting for a CAN socket operation to complete.

---

## 13. Operating Mode Is Explicit

The include tree contains:

```text
operating_mode.hpp
```

which indicates that operating mode is treated as a system-level concept rather than as a collection of unrelated boolean flags.

A mode abstraction is important because the Jetson application can have different responsibilities depending on whether the system is:

```text
idle
manual
ball balancing
test / diagnostic
```

The exact set of modes should remain defined by the current implementation rather than being duplicated in documentation.

The important architectural point is:

> Mode selection belongs to system state; individual algorithms should not independently decide the global operating mode.

---

## 14. System State Is a First-Class Module

The presence of:

```text
system_state.hpp
system_state.cpp
```

creates a central state boundary for the Jetson application.

This avoids having:

```text
camera task
CAN task
control task
watchdog
```

each maintain its own independent interpretation of system health.

Instead:

```text
Subsystem events
      ↓
System state
      ↓
Control decisions
```

This is particularly useful for handling conditions such as:

```text
camera unavailable
ball invalid
CAN unavailable
STM32 state stale
```

without spreading the complete fault policy across multiple source files.

---

## 15. Watchdog on the Jetson Side

The Jetson source tree also contains:

```text
task_watchdog
```

This is important because the Jetson is not protected by the STM32 IWDG.

The two watchdogs therefore supervise different processors:

```text
Jetson watchdog
        ↓
Jetson task/application health

STM32 IWDG
        ↓
STM32 critical execution health
```

This gives the distributed architecture independent health supervision on both sides.

The STM32 watchdog cannot determine that a Jetson Linux process has deadlocked if CAN frames stop arriving.

Conversely, the Jetson watchdog cannot recover an STM32 task that has stopped executing.

The independent watchdog domains therefore complement the CAN heartbeat/freshness mechanisms described in the failsafe design.

---

## 16. End-to-End Control Path

The complete normal control path is:

```text
                 JETSON
────────────────────────────────────

Camera
  │
  ▼
CameraPipeline
  │
  ▼
Camera Capture Task
  │
  ▼
Ball Detection Task
  │
  ▼
Ball State
  │
  ▼
Control Loop
  │
  ├── target
  ├── ball position
  └── PID controller
  │
  ▼
Desired platform command
  │
  ▼
CAN TX Task
  │
  ▼
SocketCAN
  │
────────────────────────────────────
                 CAN
────────────────────────────────────
  │
  ▼
STM32 CAN RX
  │
  ▼
Control / Trajectory
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

This division is one of the key architectural characteristics of the project:

```text
Jetson = perception + high-level control
STM32  = deterministic low-level control + actuation
```

The repository itself reflects this split through separate top-level `jetson-vision-control` and `firmware-stm32h723` directories. citeturn0view0

---

## 17. Why Control Is Not Fully Moved to the STM32

A three-servo ball-balancing platform can theoretically perform simple control entirely on a microcontroller.

The architecture instead places computer vision and PID computation on the Jetson because the vision workload benefits from Linux-class CPU/GPU resources and camera-processing flexibility.

The STM32 then remains focused on tasks where deterministic embedded execution is more valuable:

```text
sensor acquisition
control timing
trajectory generation
IK
servo limits
failsafe
```

This is a system-partitioning decision rather than simply a hardware-performance decision.

---

## 18. Calibration and Runtime Processing Are Separate

The Jetson repository contains a dedicated:

```text
calib/
```

directory.

This reflects a useful separation between:

```text
calibration / configuration
```

and:

```text
runtime processing
```

The runtime vision pipeline should consume already established camera/vision parameters rather than performing expensive calibration work during every control cycle.

A conceptual lifecycle is:

```text
Camera / platform setup
        ↓
Calibration
        ↓
Stored calibration parameters
        ↓
Runtime camera pipeline
        ↓
Ball detection
```

This follows the same general engineering principle used on the STM32 side:

> expensive or state-changing calibration operations should not be mixed into the high-rate control path.

---

## 19. Test Programs Are Part of the Architecture

The Jetson source tree contains dedicated test entry points:

```text
main_j3_test.cpp
main_j4_test.cpp
main_j6_ball_test.cpp
main_j6_camera_test.cpp
main_j7_control_test.cpp
```

These are valuable because they indicate that subsystem integration was treated incrementally.

Instead of debugging the entire system simultaneously:

```text
Camera
  ↓
Ball
  ↓
Control
  ↓
CAN
  ↓
Full system
```

the repository provides separate test executables for progressively larger parts of the pipeline.

This is a practical embedded/robotics development approach because failures can be localized before full hardware integration.

---

## 20. Failure Boundaries Between Vision and Control

The most important interface is not merely:

```text
x, y
```

but:

```text
ball state
+
validity
+
timeliness
```

The control loop should not interpret:

```text
no detection
```

as:

```text
ball is at (0,0)
```

These represent fundamentally different conditions.

The intended conceptual contract is:

```text
Valid measurement
      ↓
control

Invalid / stale measurement
      ↓
controlled fallback
```

The downstream STM32 system provides an additional safety boundary through CAN freshness monitoring, trajectory fallback, and actuator limits.

This creates defense in depth across the processor boundary.

---

## 21. Jetson-to-STM32 Responsibility Boundary

The interface can be summarized as:

| Responsibility | Jetson | STM32 |
|---|:---:|:---:|
| Camera capture | ✓ | |
| Ball detection | ✓ | |
| Ball-state processing | ✓ | |
| High-level PID | ✓ | |
| CAN protocol TX/RX | ✓ | ✓ |
| IMU acquisition | | ✓ |
| Trajectory generation | | ✓ |
| Inverse kinematics | | ✓ |
| Servo command limiting | | ✓ |
| PWM generation | | ✓ |
| Critical-task watchdog | ✓ | ✓ |
| Hardware IWDG | | ✓ |

This division prevents the Linux application from becoming the only layer capable of maintaining safe actuator behavior.

Even if the Jetson sends an invalid or stale target, the STM32 remains the final authority over the physical actuator command.

---

## 22. Engineering Trade-Offs

### Linux/Jetson for vision and high-level control

**Advantages**

- flexible camera and OpenCV-based processing;
- easier debugging and logging;
- higher computational headroom;
- easier experimentation with control algorithms.

**Trade-offs**

- non-deterministic Linux scheduling;
- process/thread failures;
- higher software-stack complexity;
- dependency on CAN communication to the STM32.

The architecture therefore avoids putting the final actuator safety boundary on Linux.

### STM32 for low-level control

**Advantages**

- deterministic RTOS scheduling;
- direct peripheral access;
- hardware watchdog;
- predictable actuator timing;
- independent operation from the Linux application.

**Trade-offs**

- limited computational resources compared with the Jetson;
- more restrictive development environment;
- vision workloads are inappropriate for this processor.

The resulting split is a pragmatic distributed-control architecture.

---

## 23. Engineering Summary

The Jetson subsystem can be reduced to four major layers:

```text
                PERCEPTION
Camera → Capture → Ball Detection
                    │
                    ▼
                ESTIMATION
              Ball State
                    │
                    ▼
                 CONTROL
             PID / Control Loop
                    │
                    ▼
              COMMUNICATION
            CAN TX / SocketCAN
                    │
                    ▼
                STM32 H723
```

Around this path, the implementation provides separate infrastructure for:

```text
system state
watchdog
CAN RX
CAN TX
video recording
calibration
testing
```

The main architectural decision is to keep **vision, control computation, transport, and low-level actuation as separate responsibilities**.

The Jetson generates a high-level desired platform command.

The STM32 remains responsible for turning that command into a physically bounded actuator trajectory.

That boundary is important for the overall reliability of the system:

```text
Vision failure
      ↓
invalid measurement

Control failure
      ↓
invalid desired command

CAN failure
      ↓
stale communication

STM32 safety layer
      ↓
reject / replace unsafe behavior
      ↓
bounded trajectory
      ↓
bounded servo command
```

The result is a distributed architecture in which the Jetson provides computational flexibility while the STM32 retains deterministic low-level authority over the physical system.

# Vision-Based Ball Balancing Table
### STM32H723 + Jetson Orin Nano | Computer Vision | Real-Time Control | Robotics

<p align="center">
  <img src="Jetson%2BCamera%2BCAN%20module.jpg" alt="Jetson, camera and CAN integration" width="850">
</p>

<p align="center">
  <b>An end-to-end mechatronic control system combining computer vision, real-time embedded control, CAN communication, calibrated inverse kinematics, and layered failsafe mechanisms.</b>
</p>

---

## 1. Project Overview

This project is a **vision-based three-servo ball-balancing platform**.

The system combines:

- **NVIDIA Jetson Orin Nano** for camera processing, ball detection, state estimation, and high-level control;
- **STM32H723** for deterministic real-time execution, IMU acquisition, trajectory generation, inverse kinematics, servo control, and safety supervision;
- **CAN bus** as the processor-to-processor control interface;
- **three RC servos** as physical actuators;
- **MPU6500 IMU** for platform motion feedback;
- **calibrated second-order inverse kinematics** for the servo mapping;
- **FreeRTOS** for concurrent embedded tasks;
- **persistent Flash calibration data** with versioning and CRC validation.

The central engineering principle is:

> **Use the Jetson where computational flexibility is valuable, while keeping final real-time actuator authority on the STM32.**

This makes the project more than a computer-vision demo: it is a complete **perception → estimation → control → communication → actuation → safety** pipeline.

---

## 2. System at a Glance

```text
                       CAMERA
                         │
                         ▼
               ┌────────────────────┐
               │  JETSON ORIN NANO  │
               │                    │
               │ Camera Capture     │
               │ Ball Detection     │
               │ Ball State         │
               │ PID / Control      │
               └─────────┬──────────┘
                         │
                         │ CAN
                         ▼
               ┌────────────────────┐
               │     STM32H723      │
               │                    │
               │ CAN RX             │
               │ IMU / SPI DMA      │
               │ State Machine      │
               │ Trajectory Engine  │
               │ Inverse Kinematics │
               │ Servo Actuator     │
               │ Failsafe / IWDG    │
               └─────────┬──────────┘
                         │
                  ┌──────┼──────┐
                  ▼      ▼      ▼
                Servo 1 Servo 2 Servo 3
                  └──────┼──────┘
                         ▼
                 BALANCING TABLE
                         │
                         ● Ball
```

### Processor responsibility boundary

**Jetson — Perception & High-Level Control**

```text
Camera → Capture → Ball Detection → Ball State → Control → CAN
```

**STM32 — Real-Time Control & Physical Safety**

```text
CAN → Target → Trajectory → IK → Servo Constraints → PWM
```

The Jetson determines the desired platform behavior. The STM32 remains the final control boundary between software commands and the physical actuators.

---

## 3. Engineering Architecture

The project is deliberately split into two processors with different responsibilities.

### Jetson

- camera acquisition;
- ball detection;
- ball-state processing;
- high-level PID/control;
- CAN communication;
- video recording and Linux-side diagnostics.

### STM32H723

- FreeRTOS task execution;
- MPU6500 acquisition through SPI/DMA;
- sensor fusion;
- CAN protocol handling;
- state machine;
- trajectory generation;
- second-order inverse kinematics;
- servo dynamic limits;
- PWM output;
- calibration persistence;
- watchdog and failsafe handling.

This separation avoids making Linux scheduling part of the final actuator-safety path.

---

## 4. Main Engineering Challenges

### 4.1 Distributed real-time control

The feedback loop spans:

```text
Jetson / Linux
      ↕
     CAN
      ↕
STM32 / FreeRTOS
```

The design therefore has to account for communication latency, stale data, asynchronous tasks, and processor failure.

### 4.2 Cortex-M7 cache and DMA

The STM32H723 uses a Cortex-M7 architecture with data cache. DMA-based SPI transfers therefore require explicit cache-coherency handling.

The IMU path is organized as:

```text
IMU Data Ready
      ↓
EXTI
      ↓
SPI2 DMA
      ↓
DMA callback
      ↓
FreeRTOS notification
      ↓
IMU fusion
```

### 4.3 Nonlinear actuator mapping

The mechanical platform is not an ideal linear mechanism. The runtime IK model uses:

```text
R
P
R²
P²
R·P
1
```

for each servo, allowing the fitted model to represent nonlinear and cross-axis behavior.

### 4.4 Fault containment

Communication loss, stale vision data, peripheral faults, task stalls, and actuator-limit violations are handled at different layers rather than through one generic emergency routine.

---

## 5. Calibration and Inverse Kinematics

The calibration layer stores:

```text
Servo neutral positions
Servo limits
Servo deadband
Roll/Pitch offsets
Second-order IK coefficients
CRC32
Version information
```

The runtime mapping is conceptually:

```text
Roll + Pitch
     ↓
[R, P, R², P², R·P, 1]
     ↓
IK coefficients
     ↓
S1 / S2 / S3
     ↓
Servo actuator limits
     ↓
PWM
```

Calibration is persisted in STM32 internal Flash and validated through:

```text
Magic check
    ↓
Version check
    ↓
CRC32
    ↓
Read-back verification
    ↓
Valid calibration
```

Invalid or obsolete calibration data is not silently accepted as active runtime configuration.

---

## 6. Trajectory-Controlled Actuation

The STM32 does not convert every new target directly into an instantaneous servo command.

Instead:

```text
Target
  ↓
Trajectory Planner
  ↓
Velocity / Acceleration constraints
  ↓
Inverse Kinematics
  ↓
Servo Actuator
  ↓
Hard PWM limits
```

When the target reverses, the trajectory engine can brake the current motion before accelerating toward the new target.

This produces:

```text
Target reversal
      ↓
Braking
      ↓
Velocity → 0
      ↓
Acceleration
      ↓
New target
```

The same trajectory infrastructure can be reused for controlled return-to-neutral behavior during fault handling.

---

## 7. Layered Failsafe

Safety is not implemented as a single function.

Different failure classes are handled at different layers:

```text
CAN heartbeat
      +
Application data freshness
      +
Ball validity
      +
FDCAN recovery
      +
Critical-task supervision
      +
IWDG
      +
Trajectory fallback
      +
Servo hard limits
```

### Communication/application failure

```text
Jetson data becomes stale
          ↓
Target becomes invalid
          ↓
Neutral target
          ↓
Controlled trajectory
          ↓
Bounded servo motion
```

### Critical firmware execution failure

```text
Critical task stops progressing
          ↓
Alive bit missing
          ↓
IWDG not refreshed
          ↓
MCU reset
```

The architecture therefore distinguishes a recoverable application fault from a firmware execution failure.

---

## 8. FreeRTOS Architecture

The STM32 firmware is divided into dedicated task responsibilities such as:

```text
Sensor / IMU
CAN RX
CAN TX
Control
Actuator
UI / Display
Logger
Safety
Watchdog
State Machine
```

This avoids turning the controller into one large super-loop and makes timing, synchronization, and fault ownership easier to reason about.

Critical tasks are additionally supervised by the watchdog architecture.

---

## 9. Jetson Software Architecture

The Jetson side contains explicit modules for:

```text
camera_pipeline
ball_detector
can_transport
pid_controller
system_state
task_camera_capture
task_ball_detect
task_control_loop
task_can_rx
task_can_tx
task_video_record
task_watchdog
```

The resulting pipeline is:

```text
Camera
  ↓
Capture
  ↓
Ball Detection
  ↓
Ball State
  ↓
Control Loop
  ↓
CAN TX
  ↓
STM32
```

Video recording is kept outside the core control path so encoding and disk I/O do not become fundamental feedback-loop dependencies.

---

## 10. CAN Communication

CAN provides the processor boundary:

```text
Jetson application state
        ↓
CAN protocol
        ↓
SocketCAN
        ↓
CAN bus
        ↓
STM32 FDCAN
        ↓
STM32 application state
```

The protocol definition is kept separate from Linux-specific transport handling.

This makes the communication interface easier to test and prevents the control algorithm from being tightly coupled to SocketCAN implementation details.

---

## 11. Project Documentation

The repository is accompanied by focused engineering documents:

| Document | Focus |
|---|---|
| `01-architecture.md` | Overall system architecture |
| `02-stm32h723-cache-memory.md` | Cortex-M7 memory/cache considerations |
| `03-freertos-task-design.md` | FreeRTOS task architecture |
| `04-fdcan-protocol-en.md` | CAN protocol and communication |
| `05-imu-mpu6500-spi-dma.md` | IMU, SPI DMA and estimation |
| `06-tft-ui-design.md` | Embedded TFT UI architecture |
| `07-servo-trajectory-safety.md` | Servo control and trajectory limits |
| `08-failsafe-design.md` | Fault detection and recovery |
| `09-ik-calibration.md` | IK calibration and persistent model |
| `10-jetson-vision-control.md` | Jetson vision/control pipeline |

These documents focus not only on **what the code does**, but also on **why the implementation is structured this way**.

---

## 12. Project Media

### Mechanical platform

<p align="center">
  <img src="Table%20top%20(3%20hand,%20MPU6050).jpg" alt="Three-servo balancing platform" width="720">
</p>

### Camera calibration

<p align="center">
  <img src="Calib%20camera%20(2).jpg" alt="Camera calibration" width="800">
</p>

### Jetson / Camera / CAN integration

<p align="center">
  <img src="Jetson%2BCamera%2BCAN%20module.jpg" alt="Jetson camera CAN integration" width="850">
</p>

### Demonstration

[▶ Watch the recorded demonstration: `output2.mp4`](output2.mp4)

---

## 13. Technology Stack

### Embedded

- STM32H723 / ARM Cortex-M7
- FreeRTOS
- STM32 HAL
- SPI / DMA
- FDCAN
- PWM
- Internal Flash
- IWDG
- TFT display

### Sensors and actuators

- MPU6500 IMU
- Three RC servos
- Three-servo balancing mechanism

### Linux / Robotics

- NVIDIA Jetson Orin Nano
- Linux
- C++
- Computer vision
- PID control
- SocketCAN

### Engineering methods

- Real-time task design
- Sensor acquisition and fusion
- System identification
- Calibrated polynomial IK
- Trajectory generation
- Fault containment
- Persistent configuration
- Hardware/software integration
- Experimental validation

---

## 14. What This Project Demonstrates

The strongest aspect of the project is not one isolated algorithm. It is the integration of multiple engineering layers into one physical system:

```text
Mechanical system
       +
Camera
       +
Computer vision
       +
Sensor acquisition
       +
State estimation
       +
Control
       +
Trajectory generation
       +
Inverse kinematics
       +
CAN
       +
FreeRTOS
       +
Actuator constraints
       +
Failsafe
```

Several design decisions were made to keep the system understandable and debuggable:

### Clear processor boundaries

Jetson handles computationally flexible workloads.

STM32 owns the final physical actuator path.

### Calibration as data

Mechanical behavior is represented by validated persistent calibration instead of constants scattered throughout the controller.

### Explicit validity

Freshness and validity are treated separately from numerical values.

### Layered safety

Communication loss, software faults, actuator limits, and watchdog recovery are handled independently.

### Deterministic low-level authority

The final trajectory and actuator command remain under STM32 control rather than depending directly on Linux scheduling.

---

## 15. Recommended Reading Order

For a technical reviewer or hiring manager:

```text
01 Architecture
      ↓
03 FreeRTOS
      ↓
04 FDCAN
      ↓
05 IMU
      ↓
07 Servo / Trajectory
      ↓
08 Failsafe
      ↓
09 IK Calibration
      ↓
10 Jetson Vision / Control
```

The README gives the system-level picture first; the individual documents then provide implementation-level details.

---

## 16. Project Scope

The current project covers an integrated prototype with:

- three-servo balancing hardware;
- camera-based ball observation;
- Jetson-side vision/control pipeline;
- STM32H723 real-time firmware;
- MPU6500 SPI/DMA acquisition;
- FreeRTOS task architecture;
- FDCAN communication;
- trajectory generation;
- calibrated second-order IK;
- persistent calibration storage;
- servo dynamic and physical limits;
- layered failsafe mechanisms;
- TFT UI;
- recorded experimental data/video.

The repository intentionally documents engineering constraints as well as successful implementation. Calibration and control models are valid within the experimentally established operating region and depend on the physical configuration remaining consistent with the calibration.

---

## 17. Why This Project Is Relevant to Embedded / Robotics Roles

This project crosses several layers that are often handled by different engineers:

```text
Low-level Embedded
  ├── STM32
  ├── DMA / SPI
  ├── FDCAN
  ├── PWM
  ├── Flash
  └── Watchdog

Real-Time Software
  ├── FreeRTOS
  ├── task synchronization
  ├── state machines
  └── fault handling

Robotics
  ├── computer vision
  ├── state estimation
  ├── PID
  ├── trajectory generation
  └── inverse kinematics

System Integration
  ├── Jetson
  ├── CAN
  ├── camera
  ├── sensors
  └── physical actuators
```

For Embedded, Robotics, Controls, or R&D positions, the repository demonstrates both implementation and **system-level reasoning about interfaces, timing, calibration, and failure modes**.

---

## 18. Engineering Takeaway

This project evolved from a ball-balancing mechanism into an end-to-end embedded robotics system:

> **Perception → Estimation → Control → Communication → Trajectory → IK → Actuation → Safety**

The main engineering lesson is that a physical control system is not only an algorithm.

It is the interaction between:

```text
Software
Hardware
Timing
Communication
Calibration
Control
Mechanical behavior
Failure handling
```

The repository is structured to make those relationships visible in both the source code and the engineering documentation.

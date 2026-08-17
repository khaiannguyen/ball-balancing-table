# 04 — FDCAN Protocol Between STM32H723 and Jetson Orin Nano

> Based on the actual implementation: `App/comm/can_id.h`, `App/comm/can_protocol.h`, `App/tasks/task_can_rx.c`, `App/tasks/task_can_tx.c`, and the Jetson-side `include/can_protocol.h`, `src/can_transport.cpp`, and `src/task_can_rx.cpp`.

---

## 1. Why FDCAN Is Used Between the Two Processors

The Ball Balancing Table has two processors exchanging control-relevant data:

- ball position and velocity;
- desired platform attitude;
- measured attitude;
- servo position;
- system state and liveness.

The communication path also operates in an environment containing servo PWM and other electrical noise sources.

CAN is appropriate for this interface because it provides two properties that are useful for distributed real-time control:

1. **Bus arbitration based on frame identifier priority.**
2. **Hardware-level error detection**, including CRC, ACK, and bit-stuffing checks.

The STM32H723 uses its FDCAN peripheral, but the current application is configured for **Classic CAN**:

```text
FDCAN_CLASSIC_CAN
BRS = FDCAN_BRS_OFF
```

CAN-FD is not required because all current application payloads fit within the standard 8-byte CAN data field.

The original bring-up path also used Classic CAN with an ESP32, so retaining the same frame format keeps the protocol simple during integration.

---

## 2. CAN Identifier Map

The CAN identifier definitions are shared between the STM32 and Jetson implementations.

This avoids maintaining two independently defined protocol tables.

### STM32 → Jetson

| ID | Message | DLC | Payload |
|---|---|---:|---|
| `0x100` | `ATTITUDE` | 6 | `roll`, `pitch` as `int16 × 100°`; `height` as `int16 mm` |
| `0x101` | `RATE` | 4 | `vroll`, `vpitch` as `int16 × 100°/s` |
| `0x102` | `SERVO_POS` | 6 | `S1`, `S2`, `S3` as `uint16`, PWM µs |
| `0x103` | `ROBOT_STATE` | 2 | `mode`, state bitfield |
| `0x104` | `BALL_DESIRED` | 6 | `Ballx_d`, `Bally_d`, reserved bytes |
| `0x1FF` | `HEARTBEAT_TX` | 1 | 8-bit counter |

### Jetson → STM32

| ID | Message | DLC | Payload |
|---|---|---:|---|
| `0x200` | `BALL_POS` | 4 | `x`, `y` as `int16 mm` |
| `0x201` | `BALL_VEL` | 4 | `vx`, `vy` as `int16` |
| `0x202` | `BALL_STATE` | 1 | `detected` flag |
| `0x203` | `SERVO_CALIB` | 6 | `S1c`, `S2c`, `S3c` — reserved override channel |
| `0x204` | `ATTITUDE_DESIRED` | 6 | `roll_d`, `pitch_d`, `height_d` |
| `0x2FF` | `HEARTBEAT_RX` | — | No fixed payload |

The STM32 currently accepts the range `0x100–0x2FF` through a single configured filter range. Message classification is then performed in software using the frame identifier.

A separate `0x555` identifier exists only for CAN loopback bring-up and is not part of the normal application protocol.

---

## 3. Integer Encoding Instead of Floating Point

CAN payload size is limited to 8 bytes.

For example, a `float` requires 4 bytes per value. Using scaled integers allows the protocol to represent the required control precision more efficiently.

For angular data:

```c
can_wr_i16le(&buf[0], (int16_t)(roll * 100.0f));
```

The receiver reconstructs the value:

```c
float roll = can_rd_i16le(&frame.data[0]) / 100.0f;
```

A scale factor of 100 represents angular resolution of:

```text
0.01°
```

which is sufficient for the current servo-control application.

The protocol explicitly implements little-endian serialization through byte operations rather than copying an `int16_t` object directly into the payload.

This avoids making the wire format an implicit property of CPU endianness.

---

## 4. Reserved Servo Calibration Channel

`0x203 SERVO_CALIB` is intentionally treated as a reserved channel rather than an active actuator-calibration command.

The STM32 currently decodes the three calibration values but does not apply them directly:

```c
case CAN_ID_SERVO_CALIB:
    ...
    (void)s1c;
    (void)s2c;
    (void)s3c;
    /* TODO: apply conditionally when a real calibration override mechanism exists */
    break;
```

The official calibration source is the closed-loop calibration procedure running internally on the STM32.

Directly applying calibration values received from the Jetson would allow an external command to override the internal calibration without an explicit validation or authorization state.

The current design therefore preserves the protocol field while keeping the actuator calibration path protected.

A future implementation can enable this channel under an explicit calibration state and validation mechanism.

---

## 5. Layered Communication Failsafe

The communication architecture uses two independent liveness mechanisms.

They are intentionally not reduced to one generic timeout.

```text
              Communication Health
                     │
          ┌──────────┴──────────┐
          │                     │
 Explicit heartbeat       Data-implied liveness
     200 ms                    500 ms
          │                     │
          ▼                     ▼
 Protocol/node health      Application data health
```

This separation detects different failure modes.

---

## 6. Explicit Heartbeat — 200 ms

The protocol defines:

```c
#define CAN_HEARTBEAT_TIMEOUT_MS 200u
```

The STM32 sends `0x1FF` at the CAN transmit task rate.

The Jetson responds with `0x2FF`.

The STM32 records the most recent `0x2FF` reception time. If no new heartbeat is received for more than 200 ms:

1. the fault event bit is set;
2. the state-machine fault event is queued;
3. the system transitions into its error handling path;
4. the robot-state telemetry reports the fault condition.

The explicit state-machine event is important because merely setting a status bit does not by itself guarantee a transition into the actual `ERROR` / `SAFE_MODE` state.

---

## 7. Data-Implied Liveness — 500 ms

A second mechanism monitors actual application traffic.

### STM32 monitors Jetson / camera data

Reception of:

```text
0x200 BALL_POS
0x201 BALL_VEL
```

updates the camera/data liveness timestamp.

The balance control mode requires both:

```text
ball detected
AND
camera data is recent
```

before using Jetson-derived attitude setpoints.

Therefore, if camera data stops arriving, the controller does not continue indefinitely using an old setpoint.

### Jetson monitors STM32 data

The Jetson considers STM32 data alive when recent application frames are received from:

```text
0x100 ATTITUDE
0x102 SERVO_POS
0x103 ROBOT_STATE
0x104 BALL_DESIRED
```

The Jetson control loop uses:

```text
safe_to_run =
    detected &&
    stm32_state_is_ok()
```

If STM32 communication becomes stale, the PID loop stops rather than continuing to calculate control commands from old attitude data.

---

## 8. Why Two Liveness Mechanisms

The two mechanisms provide complementary coverage.

### Case A — Dedicated heartbeat is lost, application data continues

The data-implied mechanism can still indicate that the remote application is active.

### Case B — Heartbeat continues, application data stops

The data-implied mechanism can identify that the application data path has become stale.

Therefore:

```text
Explicit heartbeat
        +
Actual application traffic
        =
Stronger fault detection
```

The two layers are intentionally independent because they observe different aspects of communication health.

---

## 9. Bus-Off Recovery

The STM32 checks FDCAN protocol status periodically.

If the controller reports Bus-Off:

```c
if (ps.BusOff) {
    HAL_FDCAN_Stop(&hfdcan1);
    HAL_FDCAN_Start(&hfdcan1);
}
```

This provides an automatic recovery path after a transient bus fault.

Without an application-level recovery step, a node that remains in Bus-Off can stop participating in communication even after the physical fault has disappeared.

The recovery mechanism is deliberately simple and contained inside the CAN receive/monitoring path.

---

## 10. Jetson SocketCAN Transport

The Jetson side uses Linux SocketCAN.

The transport layer configures a CAN raw socket and applies CAN error-frame filtering.

During code review, the initialization order in `CanTransport::open()` should be treated carefully:

```cpp
setsockopt(fd_, ...);
fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
```

The error-filter socket option is currently attempted before the socket descriptor is assigned by `socket()`.

This means the `setsockopt()` operation can fail because `fd_` has not yet been initialized with the valid socket descriptor.

The issue does not necessarily prevent normal CAN data communication because the error is only reported, but it can prevent the intended CAN error-frame reporting from being enabled.

The robust order is:

```text
socket()
   │
   ▼
fd_ assigned
   │
   ▼
setsockopt(CAN_RAW_ERR_FILTER)
   │
   ▼
bind()
```

This should be verified/fixed before relying on SocketCAN error frames for diagnostics.

---

## 11. Protocol Design Principles

### Shared identifiers

The STM32 and Jetson use the same identifier definitions to reduce protocol drift.

### Explicit serialization

Wire-format encoding and decoding are performed byte-by-byte with explicit endianness.

### Compact payloads

Scaled integers are used where the required physical resolution is known.

### Separation of telemetry and commands

STM32 telemetry occupies `0x100–0x1FF`; Jetson commands occupy `0x200–0x2FF`.

### Layered liveness

Dedicated heartbeats and real application data are monitored independently.

### Protected calibration path

Reserved calibration commands are decoded but not applied without an explicit validation mechanism.

### Automatic recovery

The STM32 has an application-level Bus-Off recovery path.

---

## 12. Engineering Summary

The FDCAN interface is treated as a defined control-system boundary rather than a generic byte transport.

```text
STM32
  │
  ├── attitude / servo / state
  │
  ▼
Classic CAN
  │
  ├── explicit heartbeat
  ├── application data
  ├── priority-based arbitration
  └── hardware error detection
  │
  ▼
Jetson
  │
  ├── ball position / velocity
  ├── desired attitude
  └── control-state decisions
```

The protocol combines compact deterministic encoding, shared message definitions, explicit liveness monitoring, and recovery behavior so that communication faults can be contained before stale data reaches the physical control loop.

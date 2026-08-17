# 03 — FreeRTOS Task Design and Lock-Free Seqlock

> Based on the actual firmware structure: `Core/Src/main.c`, `App/tasks/*.c`, `App/utils/seqlock.h`, `App/utils/system_state.c/h`, and the Jetson POSIX-thread implementation in `src/task_can_rx.cpp`, `src/task_can_tx.cpp`, and `src/task_control_loop.cpp`.

---

## 1. Task Priority Architecture

The STM32 firmware uses CMSIS-RTOS2 / FreeRTOS with priorities selected according to the consequence of a missed deadline.

| Task | Priority | Rate / Trigger | Responsibility |
|---|---|---|---|
| `ControlLoopTask` | `osPriorityRealtime` | 100 Hz / 10 ms | Main servo control loop, IK, trajectory, mode dispatch |
| `ImuFusionTask` | `osPriorityAboveNormal4` | 1 kHz | Kalman-based roll/pitch estimation |
| `CanRxTask` | `osPriorityAboveNormal4` | Incoming frames, poll ≤50 ms | CAN reception, decoding, heartbeat failsafe |
| `CanTxTask` | `osPriorityAboveNormal` | 100 Hz / 10 ms | CAN telemetry and heartbeat |
| `WatchdogTask` | `osPriorityAboveNormal` | 10 Hz / 100 ms | Conditional IWDG refresh |
| `StateMachineTask` | `osPriorityNormal` | Event-driven | BOOT/RUN/ERROR state transitions |
| `ButtonUiTask` | `osPriorityNormal` | Button events | User input and state requests |
| `DisplayTask` | `osPriorityLow` | 25 Hz / 40 ms | TFT rendering |

The priority rule is consequence-based:

> **The closer a task is to the real-time physical control and safety path, the higher its priority.**

`ControlLoopTask` is intentionally isolated at the highest application priority because it is the task that directly produces servo commands. A delay in this task can affect the mechanical behavior of the platform.

`ImuFusionTask` and `CanRxTask` share the same priority because both provide input data required by the control loop. `CanTxTask` and `WatchdogTask` are one level lower because short transmission or supervision delays do not directly alter the current control calculation.

The UI tasks remain low priority because a delayed display update affects presentation quality rather than actuator safety.

---

## 2. Watchdog Design

The watchdog supervises three critical tasks:

```c
#define ALIVE_BIT_CONTROL_LOOP   (1u << 0)
#define ALIVE_BIT_IMU_FUSION     (1u << 1)
#define ALIVE_BIT_CAN_RX         (1u << 2)

#define ALIVE_MASK_EXPECTED     (ALIVE_BIT_CONTROL_LOOP |      ALIVE_BIT_IMU_FUSION |      ALIVE_BIT_CAN_RX)
```

Every 100 ms, `WatchdogTask` atomically snapshots and clears the current alive mask using `__atomic_exchange_n()`.

The IWDG is refreshed only when all three required bits are present:

```text
100 ms supervision window
        │
        ├── ControlLoop alive
        ├── IMU Fusion alive
        └── CAN RX alive
                 │
                 ▼
        all bits present?
           │           │
          yes          no
           │           │
        refresh       do not refresh
         IWDG             │
                           ▼
                       MCU reset
```

This is deliberate: if a critical task is stalled or deadlocked, the system should reset rather than leave the actuators indefinitely holding their previous command.

### Why the failure path does not log

The watchdog deliberately avoids logging in the branch where it does not refresh the IWDG.

A blocking UART operation or a mutex used by logging could itself be part of the failure condition. Adding diagnostics inside the protection path could therefore interfere with the mechanism intended to recover the system.

`CanTxTask` and `DisplayTask` do not contribute alive bits because the expected mask intentionally contains only the three safety-critical tasks.

---

## 3. Boot-Time TFT Synchronization and Deadlock

A real multi-task initialization issue was identified around the TFT boot screen.

The problematic sequence was:

```text
ControlLoopTask starts
       │
       ▼
ScreenBoot_AddLog()
       │
       ▼
TFT_DrawText()
       │
       ├── DMA completion
       └── ScreenManager mutex

DisplayTask has not initialized TFT yet
       │
       ▼
DisplayTask later needs the same ScreenManager mutex
```

`ControlLoopTask` could reach `ScreenBoot_AddLog()` before `DisplayTask` had completed `TFT_Init()` and established the initial screen.

This created a dependency cycle:

1. `ControlLoopTask` attempted to use the TFT before the display subsystem was ready.
2. The TFT operation waited for DMA completion while holding the ScreenManager synchronization path.
3. `DisplayTask`, which was responsible for TFT initialization, later required the same synchronization path.
4. The result could be a permanent deadlock and a blank display.

### Synchronization fix

Two explicit system events were introduced:

```c
#define EVT_BIT_TFT_READY  (1u << 6)
#define EVT_BIT_BOOT_DONE  (1u << 5)
```

`DisplayTask` sets `EVT_BIT_TFT_READY` after the TFT and Boot screen have been initialized.

`ControlLoopTask` waits for this event before its first `ScreenBoot_AddLog()` operation.

`DisplayTask` waits for `EVT_BIT_BOOT_DONE` before leaving the Boot screen for Home.

The important design point is that task startup order is no longer an implicit consequence of FreeRTOS scheduling.

---

## 4. Interaction Between Synchronization and Watchdog Timing

The TFT synchronization fix exposed a second issue.

The first implementation used one long 2-second event wait. During that period, `ControlLoopTask` was outside its normal loop and therefore did not report its alive status.

Because the watchdog supervises the system every 100 ms, the 2-second wait violated an independent timing requirement.

The final implementation divides the wait into 200 ms intervals:

```c
#define TFT_READY_POLL_MS   200u
#define TFT_READY_TOTAL_MS  2000u
```

Conceptually:

```text
Wait 200 ms
    │
    ├── mark ControlLoop alive
    │
Wait 200 ms
    │
    ├── mark ControlLoop alive
    │
    ...
    │
    ▼
TFT ready or 2 s timeout
```

This preserves both requirements:

- deterministic initialization ordering;
- continuous watchdog supervision.

The broader engineering lesson is:

> **A new synchronization mechanism must be evaluated against every existing timing constraint, especially watchdog and deadline requirements.**

---

## 5. Lock-Free Seqlock for Shared System State

The project uses a seqlock for state that follows a single-writer / multiple-reader access pattern.

The core mechanism is:

```c
typedef struct {
    volatile uint32_t seq;
} seqlock_t;

static inline void seqlock_write_begin(seqlock_t *lock)
{
    lock->seq++;
    __DMB();
}

static inline void seqlock_write_end(seqlock_t *lock)
{
    __DMB();
    lock->seq++;
}
```

A reader records the sequence number, copies the state, then verifies that the sequence number has not changed:

```text
Writer                         Reader

seq = odd
   │                              │
   │                        read seq
   │                              │
write state                        │
   │                              │
seq = even                         │
                                  ▼
                         read seq again
                              │
                     ┌────────┴────────┐
                   same             changed
                     │                  │
                 accept             retry
```

If the writer modifies the state while the reader is copying it, the reader retries rather than using a potentially torn snapshot.

---

## 6. Why Seqlock Instead of a Mutex

The following state objects use a single writer:

| State | Writer | Typical readers |
|---|---|---|
| `imu_state_t` | `ImuFusionTask` | Control, UI, telemetry |
| `ball_state_t` | `CanRxTask` | Control, UI, telemetry |
| `actuator_state_t` | `ControlLoopTask` | UI, telemetry |

For this access pattern, a seqlock avoids blocking the real-time writer.

With a mutex:

```text
Low-priority reader
        │
        ▼
      mutex
        │
        │ high-priority writer needs state
        ▼
     writer waits
```

FreeRTOS priority inheritance reduces the impact of priority inversion, but the writer still experiences synchronization overhead and potential blocking.

With a seqlock:

```text
High-priority writer
        │
        ├── increment sequence
        ├── write state
        └── increment sequence

No blocking operation
```

The reader absorbs the cost if it happens to overlap the write.

This is appropriate because the writers are the timing-sensitive tasks:

- IMU fusion at 1 kHz;
- control loop at 100 Hz.

The trade-off is that readers may retry under contention. For these small state structures, that is preferable to blocking the real-time writer.

---

## 7. Why Setpoints Use a Mutex

`setpoint_t` is intentionally different.

It has multiple writers:

- `CanRxTask` updates `Roll_d`, `Pitch_d`, and `Height_d`;
- `ButtonUiTask` updates `Ballx_d` and `Bally_d`.

A seqlock assumes a single writer and therefore is not the correct primitive for this case.

The project instead uses `SetpointMutexHandle` with a short timeout.

```text
CAN RX ───────┐
              ├──► Setpoint ───► ControlLoop
Button UI ────┘
```

The write frequency is much lower than the IMU and servo control rates, so the short mutex critical section is an acceptable trade-off.

The synchronization primitive is therefore selected according to the actual ownership model rather than applied uniformly across all shared state.

---

## 8. Jetson Scheduling Model

The Jetson does not use FreeRTOS. It runs Linux and uses POSIX threads with `SCHED_FIFO` priorities.

| Thread | `SCHED_FIFO` Priority | Responsibility |
|---|---:|---|
| `TaskControlLoop` | 80 | Real-time PID control |
| `TaskCanRx` | 70 | STM32 data reception |
| `TaskCanTx` | 70 | STM32 command transmission |

The scheduling principle remains the same as on the STM32:

```text
ControlLoop
    80
     │
     ├── highest real-time requirement
     │
CAN RX / TX
    70
     │
     └── communication support
```

The control loop is intentionally given the highest real-time priority.

---

## 9. Jetson CPU Starvation Issue

A real scheduling issue was observed during CAN communication testing.

`TaskCanRx` initially ran with the default `SCHED_OTHER` policy while `TaskControlLoop` was running under `SCHED_FIFO`.

Symptoms included:

- `stm32_ok` becoming false for more than 500 ms;
- `candump` showing that CAN traffic was still present;
- no corresponding socket or `poll()` failure.

The issue was therefore not initially consistent with a CAN transport failure.

The root cause was scheduling starvation:

```text
TaskControlLoop
SCHED_FIFO / priority 80
        │
        │ continuous CPU demand
        ▼
Linux scheduler
        │
        ├── TaskCanRx: SCHED_OTHER
        └── TaskBallDetect: CPU-intensive OpenCV
```

On the available CPU resources, the lower-priority `SCHED_OTHER` CAN receiver was not scheduled often enough to service the socket within the expected timing window.

The fix was to move CAN RX and TX to:

```text
SCHED_FIFO / priority 70
```

This keeps them below the control loop while ensuring that communication threads are not starved by normal-priority workloads.

---

## 10. Design Principles

The task architecture follows several principles:

### Priority reflects physical consequence

Servo control and safety-critical input processing receive higher priority than UI and presentation tasks.

### Watchdog ownership is explicit

Only tasks that are required for safe operation contribute to the watchdog health mask.

### Synchronization follows data ownership

- Single-writer / multiple-reader state → seqlock.
- Multiple-writer configuration → mutex.

### Initialization dependencies are explicit

Cross-task hardware dependencies are synchronized with event flags rather than relying on scheduler timing.

### Blocking is kept out of the real-time path

High-rate writers avoid mutex-based blocking where a lock-free snapshot mechanism is sufficient.

### Linux scheduling is treated as part of the control architecture

The Jetson side uses explicit `SCHED_FIFO` priorities for the control and CAN threads because communication timing can otherwise be affected by CPU starvation.

---

## 11. Engineering Summary

The FreeRTOS architecture is not based only on task frequency. It combines:

```text
Task priority
     +
Deadline sensitivity
     +
Watchdog supervision
     +
Explicit initialization ordering
     +
Data ownership
     +
Synchronization strategy
```

This allows the real-time control path to remain independent from lower-priority UI and presentation workloads while still providing bounded communication and recovery behavior.

The same scheduling principle is applied across the heterogeneous system: **the control path receives the strongest scheduling guarantees, while supporting functions are prevented from blocking or starving it.**

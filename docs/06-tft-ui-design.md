# 06 — TFT UI Design: ILI9225 + ScreenManager

> Technical design note for the Ball Balancing Table firmware.  
> Scope: STM32H723 TFT UI architecture, ILI9225 graphics service, screen state management, real-time data presentation, button navigation, concurrency protection, and display-performance optimizations.
>
> This document describes the implementation currently present in `firmware-stm32h723/App/`. It intentionally separates **implemented behavior** from **future improvements**.

---

## 1. Purpose

The TFT is the local operator/debug interface of the STM32H723 side of the Ball Balancing Table.

It has two responsibilities:

1. **Operator interface**
   - Select operating modes.
   - Adjust Position-mode ball setpoints.
   - Operate the Manual mode.
   - Start/stop the robot.
   - Confirm shutdown.

2. **Real-time diagnostic display**
   - RUN/STOP state.
   - IMU roll/pitch.
   - Ball position and velocity.
   - Camera heartbeat state.
   - Actual servo PWM values `S1/S2/S3`.
   - Boot and shutdown progress.
   - Fault indication.

The design deliberately separates these responsibilities into three layers:

```text
Application / Tasks
        │
        ▼
┌───────────────────────────────┐
│        UI / Screen Layer      │
│                               │
│ ScreenManager                 │
│ screen_gauge_common           │
│ screen_boot / fault /         │
│ shutdown / ...                │
└───────────────┬───────────────┘
                │
                │ TFT graphics API
                ▼
┌───────────────────────────────┐
│       TFT Service Layer       │
│                               │
│ ILI9225 initialization        │
│ SPI1                         │
│ DMA                          │
│ primitives                    │
│ text rendering                │
│ dirty/fast drawing support    │
└───────────────────────────────┘
                │
                ▼
             ILI9225
            220 × 176
```

The main implementation files are:

```text
firmware-stm32h723/
└── App/
    ├── bsp/
    │   ├── tft_service.c
    │   └── tft_service.h
    │
    ├── UI/
    │   ├── screen.h
    │   ├── screen_manager.c
    │   ├── screen_gauge_common.c
    │   ├── screen_home.c
    │   ├── screen_calibrate.c
    │   ├── screen_balance.c
    │   ├── screen_position.c
    │   ├── screen_manual.c
    │   ├── screen_boot.c
    │   ├── screen_shutdown.c
    │   ├── screen_fault.c
    │   ├── screen_stop.c
    │   ├── ui_data.c
    │   └── ui_data.h
    │
    └── tasks/
        ├── task_display.c
        └── task_button_ui.c
```

---

# 2. Display Hardware and Coordinate System

The TFT service defines:

```c
#define TFT_WIDTH       220
#define TFT_HEIGHT      176
#define TFT_PHYS_WIDTH  176
#define TFT_PHYS_HEIGHT 220
```

The panel is therefore physically `176 × 220`, but the application uses a logical landscape coordinate system of `220 × 176`.

The logical coordinate system is:

```text
(0,0)
  ┌──────────────────────────────────────────────┐
  │                                              │
  │              220 pixels                      │
  │                                              │
  │                                              │
  │                                              │
  │              176 pixels                      │
  │                                              │
  └──────────────────────────────────────────────┘
                                             (219,175)
```

`TFT_SetWindow()` converts logical `(x,y)` coordinates into the physical ILI9225 addressing scheme.

The current transformation is:

```text
physical_x = logical_y
physical_y = (PHYS_HEIGHT - 1) - logical_x
```

This allows the rest of the UI code to remain independent of the physical panel orientation.

---

# 3. TFT Low-Level Architecture

## 3.1 SPI interface

The TFT uses SPI1:

```c
extern SPI_HandleTypeDef hspi1;
#define TFT_SPI (&hspi1)
```

The ILI9225 GRAM write command is:

```c
#define ILI9225_GRAM_WRITE 0x22
```

The service contains low-level operations for:

- register write;
- GRAM write;
- address-window setup;
- pixel;
- line;
- rectangle;
- circle;
- text;
- DMA transfer.

The UI layer does not access SPI registers directly. It calls the TFT service API.

This is an important architectural boundary:

```text
Screen code
    │
    ├── TFT_DrawTextFast()
    ├── TFT_FillRectangle()
    ├── TFT_FillCircleFast()
    └── TFT_DrawLine()
            │
            ▼
       TFT service
            │
            ▼
          SPI1 DMA
            │
            ▼
         ILI9225
```

---

# 4. SPI DMA Completion Synchronization

A key problem encountered during development was how the display task should wait for SPI DMA completion.

The early implementation used a flag and/or yielding loop. That was problematic because the display task could remain in the Ready state and prevent lower-priority system services from getting CPU time.

The current design uses a semaphore.

Conceptually:

```text
Task_Display
    │
    │ HAL_SPI_Transmit_DMA()
    ▼
 SPI1 DMA transfer
    │
    │ DMA complete interrupt
    ▼
HAL_SPI_TxCpltCallback()
    │
    │ give semaphore from ISR
    ▼
Display task wakes immediately
```

This has two advantages:

1. The task is actually blocked while DMA is running.
2. It wakes immediately when the transfer completes instead of waiting for the next RTOS tick.

The implementation deliberately uses the ISR-safe semaphore path for DMA completion rather than treating the callback like normal task context.

### Why this matters

The TFT is not a real-time control device, but the display task shares the MCU with:

- control loop;
- IMU fusion;
- CAN RX;
- button processing;
- watchdog-related tasks.

Therefore, a display operation must not monopolize CPU time simply because it is performing graphics.

---

# 5. D-Cache and DMA Coherency

The STM32H723 is a Cortex-M7 with D-Cache.

The TFT service therefore cleans the D-Cache before DMA reads buffers from RAM.

For example:

```c
SCB_CleanDCache_by_Addr(
    (uint32_t*)lineBuffer,
    sizeof(lineBuffer));
```

and for text sprites:

```c
SCB_CleanDCache_by_Addr(
    (uint32_t*)textSpriteBuf,
    (uint32_t)w * h * sizeof(uint16_t));
```

The reason is:

```text
CPU writes buffer
      │
      ▼
D-Cache contains newest data
      │
      X
      │ DMA cannot read CPU cache
      │
      ▼
RAM may still contain old data
      │
      ▼
DMA sends stale pixels
```

Cleaning the cache makes the CPU-written buffer visible in RAM before DMA starts:

```text
CPU writes buffer
      │
      ▼
Clean D-Cache
      │
      ▼
RAM contains newest pixels
      │
      ▼
SPI DMA
      │
      ▼
ILI9225
```

This is especially important for the text sprite buffer because it contains different pixel values rather than one repeated color.

---

# 6. Graphics Primitive Design

The TFT service exposes basic graphics primitives:

```c
TFT_DrawPixel()
TFT_DrawLine()
TFT_DrawRectangle()
TFT_FillRectangle()
TFT_DrawCircle()
TFT_FillCircle()
TFT_FillCircleFast()
TFT_DrawChar()
TFT_DrawText()
TFT_DrawTextFast()
TFT_GetTextExtent()
```

The normal drawing functions are intentionally simple and useful as a fallback.

For high-frequency UI updates, faster functions are used.

---

# 7. Why `TFT_DrawTextFast()` Exists

The original text renderer draws characters using many individual graphics operations.

For a 5×7 font, this can result in many SPI transactions for a single string.

That creates a visible problem on the ILI9225:

```text
draw character 1
draw character 2
draw character 3
...
```

The display can temporarily show a partially updated UI.

The optimized renderer instead creates a complete text sprite in RAM:

```text
String
  │
  ▼
5×7 glyph conversion
  │
  ▼
RAM sprite buffer
  │
  ▼
D-Cache clean
  │
  ▼
SetWindow()
  │
  ▼
ONE SPI DMA transfer
  │
  ▼
ILI9225
```

The sprite buffer is:

```c
static uint16_t textSpriteBuf[
    TEXT_SPRITE_MAX_W * TEXT_SPRITE_MAX_H
] __attribute__((aligned(32)));
```

The 32-byte alignment is appropriate for Cortex-M7 cache-line operations.

The current sprite size is based on:

```c
#define TEXT_SPRITE_MAX_W TFT_WIDTH
#define TEXT_SPRITE_MAX_H (FONT_H * 3)
```

The function also checks the requested size and screen boundaries. If the request is too large, it safely falls back to `TFT_DrawText()` instead of overflowing the buffer.

---

# 8. Why Text Can Appear Mirrored

The ILI9225 entry mode and the logical-to-physical rotation create an important detail for text.

A solid-color rectangle can appear correct even when pixel ordering is wrong because every pixel has the same color.

Text is different:

```text
A B C
```

contains different pixels, so incorrect memory ordering becomes visible as mirrored or reversed text.

The current implementation therefore has:

```c
#define TEXT_SPRITE_FLIP_MODE 1
```

The sprite is horizontally transformed before DMA transmission.

This is an example of a hardware-specific display issue that is hidden from the higher-level screen code.

---

# 9. UI Data Architecture

The UI does not directly read every subsystem whenever it draws.

Instead, `RobotUiData_t` provides a display-oriented snapshot:

```c
typedef struct
{
    uint8_t  mode;
    bool     robotRunning;

    float imuRoll;
    float imuPitch;

    bool ballOn;
    float ballX;
    float ballY;
    float ballVx;
    float ballVy;

    bool cameraOk;

    int16_t servoUs[3];

    char guideText[48];
} RobotUiData_t;
```

The design intention is:

```text
Control / CAN / IMU / actuator state
              │
              ▼
   UiData_SyncFromSystemState()
              │
              ▼
          g_uiData
              │
              ▼
       ScreenManager_Update()
              │
              ▼
             TFT
```

`Task_Display` calls:

```c
UiData_SyncFromSystemState();
```

before drawing.

This means the screen layer is primarily a presentation layer rather than another owner of the robot state.

---

# 10. What Data Is Actually Displayed

The current UI synchronization reads:

### IMU

```text
Roll
Pitch
```

### Ball

```text
Ball detected ON/OFF
X
Y
Vx
Vy
```

### Camera

```text
OK / OFF
```

The distinction is important:

```text
ballOn
    = camera currently reports a detected ball

cameraOk
    = camera/CAN heartbeat is still alive
```

Therefore:

```text
cameraOk = true
ballOn   = false
```

is a valid state: the camera is alive but the ball is not currently detected.

### Actuators

The UI displays the actual servo values:

```text
S1
S2
S3
```

in microseconds.

This is more useful for debugging than a simple `OK/ERROR` indicator because it exposes the actual actuator command reaching the UI state layer.

---

# 11. Screen Architecture

Every screen implements the same interface:

```c
typedef struct Screen
{
    void (*onEnter)(void);
    void (*onExit)(void);
    void (*update)(void);
    void (*onButton)(ButtonState_t evt);
} Screen_t;
```

This gives each screen four responsibilities:

| Callback | Responsibility |
|---|---|
| `onEnter()` | Draw/reset screen state when entering |
| `onExit()` | Cleanup before leaving |
| `update()` | Periodic dynamic rendering |
| `onButton()` | Handle user input |

The UI therefore follows a small state-machine-like architecture:

```text
                 ┌──────────────┐
                 │ ScreenManager│
                 └──────┬───────┘
                        │
       ┌────────────────┼────────────────┐
       ▼                ▼                ▼
   onEnter()         update()        onButton()
       │                │                │
       └────────────────┴────────────────┘
                        │
                        ▼
                      TFT
```

---

# 12. ScreenManager

The current ScreenManager maintains:

```c
static const Screen_t *currentScreen;
```

and a stack:

```c
static const Screen_t *screenStack[SCREEN_STACK_DEPTH];
```

with:

```c
#define SCREEN_STACK_DEPTH 4
```

The stack supports nested screens.

Example:

```text
Balance
   │
   ├── Shutdown
   │      │
   │      └── Fault
   │
   ▼
GoBack()
   │
   ▼
Shutdown
```

This is safer than a single `rememberedScreen` variable because nested transitions do not overwrite the previous navigation context.

---

# 13. ScreenManager Navigation API

The public API is:

```c
ScreenManager_Goto()
ScreenManager_GotoAndRemember()
ScreenManager_GoBack()
ScreenManager_Update()
ScreenManager_OnButton()
```

Their intended semantics are:

### `Goto`

Replace the current screen.

```text
A → B
```

### `GotoAndRemember`

Push the current screen and enter another screen.

```text
A → B
stack = [A]
```

### `GoBack`

Pop the previous screen.

```text
B → A
stack = []
```

### `Update`

Call the active screen's periodic update.

### `OnButton`

Route a button event to the active screen.

---

# 14. The TFT Race Condition That Required a Mutex

There are two important UI execution contexts:

```text
Task_Display
    └── ScreenManager_Update()
            └── screen.update()
                    └── TFT drawing

Task_Button_UI
    └── ScreenManager_OnButton()
            └── screen.onButton()
                    └── TFT drawing / navigation
```

Both can potentially write to the same SPI/TFT device.

Without synchronization:

```text
Task_Display                  Task_Button_UI
     │                              │
     ├── draw old screen            │
     │                              ├── enter new screen
     │                              │
     ├── draw text                  ├── draw shutdown
     │                              │
     └──────── SPI writes overlap ──┘
```

The observed symptom was mixed screen content: parts of the old gauge screen and parts of the Shutdown screen appeared together.

The fix is a single mutex around the ScreenManager public operations.

---

# 15. Why the Mutex Is Recursive

A normal mutex would create a deadlock in this design.

For example:

```text
ScreenManager_OnButton()
    │
    ├── Lock()
    │
    └── screen.onButton()
             │
             └── ScreenManager_Goto()
                     │
                     └── Lock() again
```

The same task would attempt to acquire the same mutex twice.

A normal mutex would deadlock itself.

The current implementation therefore creates:

```c
osMutexAttr_t s_screenMutexAttr =
{
    .name = "screenMutex",
    .attr_bits = osMutexRecursive
};
```

This allows:

```text
same task:
Lock
  └── Lock
      └── Unlock
  └── Unlock
```

while still preventing two different tasks from drawing simultaneously.

This is an important RTOS/UI design decision: the recursive property solves re-entrant navigation while the mutex itself provides cross-task exclusion.

---

# 16. Display Task Timing

`Task_Display` runs at:

```c
#define DISPLAY_PERIOD_MS (1000 / 25)
```

therefore:

```text
25 Hz
≈ 40 ms period
```

The loop is:

```text
UiData_SyncFromSystemState()
        │
        ▼
fault edge detection
        │
        ▼
ScreenManager_Update()
        │
        ▼
osDelayUntil()
        │
        └──────────► next 25 Hz cycle
```

The task uses `osDelayUntil()` rather than accumulating delay time.

This is important because:

```text
osDelay(40)
```

would make the actual period depend on execution time:

```text
render 8 ms + delay 40 ms = 48 ms
```

while the `DelayUntil` model targets a fixed periodic schedule.

---

# 17. Boot Screen

The boot screen is a dedicated screen:

```text
screen_boot.c
```

Its layout is:

```text
PINGPONG-TABLE: BOOT........

┌──────────────────────────────────────────────┐
│ init log                                     │
│ init log                                     │
│ init log                                     │
│ ...                                          │
└──────────────────────────────────────────────┘
```

It stores up to:

```c
#define BOOT_MAX_LINES 12
#define BOOT_LINE_LEN  40
```

When the buffer is full, the oldest line is removed and the remaining lines are shifted upward.

`ScreenBoot_AddLog()` is protected by the ScreenManager mutex because boot logs can originate from a task different from `Task_Display`.

---

# 18. Boot Synchronization

`Task_Display` does not simply sleep for an arbitrary fixed boot time.

The current sequence is:

```text
TFT_Init()
    │
    ▼
UiData_Init()
    │
    ▼
ScreenBoot
    │
    ▼
EVT_BIT_TFT_READY
    │
    ▼
wait EVT_BIT_BOOT_DONE
    │
    ▼
ensure minimum boot display time
    │
    ▼
ScreenHome
```

The minimum boot display time is:

```c
#define BOOT_MIN_DISPLAY_MS 1500u
```

The important design distinction is:

```text
BOOT_DONE
    = initialization actually completed

BOOT_MIN_DISPLAY_MS
    = human-readable minimum time
```

They are not used as substitutes for each other.

---

# 19. Main Gauge Screen Architecture

The five operating screens currently share one common implementation:

```c
ScreenGauge_Get(...)
```

The modes are:

```text
MODE_HOME       = 0
MODE_CALIBRATE = 1
MODE_BALANCE   = 2
MODE_POSITION  = 3
MODE_MANUAL    = 4
```

The wrapper files are intentionally thin:

```text
screen_home.c
screen_calibrate.c
screen_balance.c
screen_position.c
screen_manual.c
```

For example:

```c
return ScreenGauge_Get(MODE_BALANCE, "2. Balance");
```

This avoids duplicating the same rendering logic five times.

---

# 20. Gauge Screen Layout

The logical display is divided into:

```text
┌─────────────────────────────────────────────────────────────┐
│ PINGPONG-TABLE                         MODE / NAV / SEL     │
│                                                             │
│ State: RUN/STOP          ┌─────────────────────────────┐    │
│ Roll:                    │                             │    │
│ Pitch:                   │       circular table        │    │
│ Ball:                    │                             │    │
│ x:                       │      + crosshair            │    │
│ y:                       │       + hexagon             │    │
│ vx:                      │        + ball                │    │
│ vy:                      │                             │    │
│ Cam:                     └─────────────────────────────┘    │
│ S1:                                                         │
│ S2:                         instructions:                    │
│ S3:                         <guide text>                     │
│                                                             │
│ ●  ●  ●        [ current mode ]       [ Shutdown ]          │
└─────────────────────────────────────────────────────────────┘
```

The current implementation defines:

```c
CIRCLE_CX = 145
CIRCLE_CY = 75
CIRCLE_R  = 72
```

The ball is mapped into the circle using:

```c
BALL_PHYS_MAX = 178.0f
```

and the coordinate mapping:

```text
pixel_x = CIRCLE_CX + x / BALL_PHYS_MAX * CIRCLE_R

pixel_y = CIRCLE_CY - y / BALL_PHYS_MAX * CIRCLE_R
```

The Y axis is inverted because screen coordinates increase downward.

---

# 21. Table Visualization

The gauge screen draws three static elements:

1. Circular table boundary.
2. White X/Y crosshair.
3. Red regular hexagon.

The hexagon is computed once:

```c
ComputeHexVertices()
```

and cached in:

```c
s_hexX[6]
s_hexY[6]
```

This avoids repeated trigonometric calculations every 25 Hz update.

The hexagon uses:

```text
radius = 55 pixels
angle step = 60°
```

with the first vertex at `-90°`.

This is a good example of separating static geometry from dynamic rendering.

---

# 22. Dirty Rendering

A major display optimization is the dirty-update strategy.

The gauge screen does **not** redraw the complete screen every 25 Hz.

Instead:

```text
static background
    ├── table circle
    ├── crosshair
    ├── hexagon
    ├── labels
    └── buttons
```

is drawn when entering the screen.

Then:

```text
dynamic values
    ├── State
    ├── Roll
    ├── Pitch
    ├── Ball
    ├── x/y
    ├── vx/vy
    ├── camera
    ├── S1/S2/S3
    └── guide text
```

are updated only when their displayed value changes.

A cache structure stores the previous displayed state:

```c
typedef struct
{
    bool valid;
    uint8_t mode;
    uint8_t navBrowse;
    uint8_t robotRunning;
    int16_t imuRoll;
    int16_t imuPitch;
    uint8_t ballOn;
    int16_t ballXShown;
    int16_t ballYShown;
    uint8_t showDesired;
    int16_t ballVx;
    int16_t ballVy;
    uint8_t cameraOk;
    int16_t servoUs[3];
    char guideText[64];
} UiCache_t;
```

The result is:

```text
Old value == new value
        │
        └── no SPI write

Old value != new value
        │
        └── redraw only that field
```

This is particularly important for the display because SPI transactions are much more expensive than normal RAM comparisons.

---

# 23. Dirty Ball Rendering

The ball is treated separately from the text fields.

The screen stores:

```c
lastBallPx
lastBallPy
ballWasDrawn
```

If the ball pixel position has not changed:

```text
no drawing
```

If it moved:

```text
erase old ball
      │
      ▼
repair static geometry underneath
      │
      ▼
draw new ball
```

This avoids redrawing the entire table just because the ball moved.

---

# 24. Repairing Geometry Under the Ball

Simply erasing the old ball with background color would destroy any static graphic underneath it.

For example:

```text
hexagon
   │
   ● ball
   │
erase ball
   │
   X
```

could leave a gap in the hexagon or crosshair.

The current implementation therefore uses:

```c
RepairStaticNear(lastBallPx, lastBallPy);
```

It checks whether the old ball overlapped:

- circular boundary;
- horizontal crosshair;
- vertical crosshair;
- one or more hexagon edges.

Only affected geometry is redrawn.

This is more efficient than:

```text
erase ball
redraw complete screen
draw ball
```

---

# 25. Position Mode

Position mode has a special UI behavior.

When:

```text
activeMode == MODE_POSITION
navState == NAV_SELECTED
```

the X/Y values shown on screen represent the desired ball position:

```text
ballXDesired
ballYDesired
```

rather than the measured ball position.

In other modes, X/Y display the measured ball position from `g_uiData`.

This avoids a common operator-interface problem:

```text
user presses RIGHT
      │
      ▼
desired X changes
      │
      ▼
screen immediately shows desired X
```

instead of forcing the user to infer the new setpoint from a delayed physical response.

The desired position is clamped to:

```c
[-BALL_PHYS_MAX, +BALL_PHYS_MAX]
```

and published through `setpoint_set()`.

---

# 26. Navigation Model: SELECTED vs BROWSE

The gauge screens use two navigation states:

```c
NAV_SELECTED
NAV_BROWSE
```

### SELECTED

The current mode is actually committed to the robot.

Examples:

```text
LEFT / RIGHT
    → Position mode: modify X

UP / DOWN
    → Position mode: modify Y
```

### BROWSE

The user is selecting another mode.

```text
LEFT
    → previous mode

RIGHT
    → next mode
```

The important design decision is that browsing does not immediately change the actual robot mode.

The actual mode is committed only when:

```text
SELECTED
    │ ENTER
    ▼
BROWSE
    │
    │ choose mode
    │
    │ ENTER
    ▼
SELECTED
    │
    ▼
setpoint.mode = activeMode
```

This prevents an accidental LEFT/RIGHT button press from immediately commanding a different control mode.

---

# 27. Button Mapping

The physical button mapping is defined in `screen.h` and implemented by `Task_Button_UI`.

The current mapping is:

| Physical input | UI event |
|---|---|
| BTN_ID_1 short | ENTER |
| BTN_ID_1 long | RUN/STOP handling outside ScreenManager |
| BTN_ID_2 | UP |
| BTN_ID_3 | LEFT |
| BTN_ID_4 | DOWN |
| BTN_ID_5 | RIGHT |
| BTN_ID_6 | EXIT |

The long press is intentionally treated differently from normal navigation.

It is an operational safety action, not a screen-navigation event.

---

# 28. Emergency STOP Overlay

The original architecture had a separate Stop screen.

The current gauge architecture instead uses an overlay:

```text
normal gauge screen
       │
       │ BTN1 long
       ▼
same gauge screen
       │
       └── circular red STOP overlay
```

The reason is useful for diagnostics:

```text
STOP
+
Roll/Pitch
+
Ball
+
S1/S2/S3
+
Camera state
```

remain conceptually associated with the same gauge screen.

The overlay is drawn using:

```c
TFT_FillCircleFast()
TFT_DrawTextFast()
```

and the underlying ball renderer is disabled while the overlay is active.

When the overlay is cleared, the static gauge is rebuilt and the dynamic fields are forced to redraw.

---

# 29. Manual Mode

Manual mode reuses the gauge screen but changes button semantics.

When:

```text
MODE_MANUAL
+
NAV_SELECTED
```

the direction buttons are routed to:

```c
HandleManualButton()
```

The current manual behavior includes:

```text
LEFT / RIGHT
    → servo adjustment in ±10 µs steps

UP / DOWN
    → select/cycle servo channel
```

Automatic manual-mode tools can also use the same UI state machine.

This keeps the UI consistent without creating a completely separate rendering implementation.

---

# 30. Shutdown Screen

The Shutdown screen is a two-state UI:

```text
SD_STATE_CONFIRM
SD_STATE_LOGGING
```

Initial state:

```text
┌─────────────────────────────────────┐
│ Back to HOME and shut down?         │
│                                     │
│      [ YES ]      [ NO ]             │
│                                     │
├─────────────────────────────────────┤
│ shutdown log                        │
│ ...                                 │
└─────────────────────────────────────┘
```

The current code:

- ENTER transitions to `SD_STATE_LOGGING`.
- EXIT returns to the previous screen.

The actual actuator/safety shutdown procedure is still marked as a TODO in the current implementation.

Therefore, the screen should be described as:

> shutdown workflow UI / confirmation layer

rather than claiming that it owns the complete physical shutdown sequence.

---

# 31. Fault Screen

The Fault screen is deliberately simple:

```text
FAULT

Check CAN and sensor signal
```

The current `system_state` interface exposes a fault condition but does not yet expose a detailed fault message.

Therefore the current screen cannot distinguish, for example:

```text
CAN heartbeat timeout
```

from:

```text
sensor fault
```

at the text-display level.

A future extension could add:

```c
fault_code
fault_message
fault_timestamp
```

to the shared system state.

This should be implemented only if the system-level fault architecture requires it.

---

# 32. Fault Navigation

`Task_Display` detects a rising/falling fault edge:

```text
no fault → fault
```

causes:

```c
ScreenManager_GotoAndRemember(ScreenFault_Get());
```

When the fault clears:

```text
fault → no fault
```

the screen manager calls:

```c
ScreenManager_GoBack();
```

Because the navigation mechanism uses a stack, the system can return to the screen that was active before the fault.

---

# 33. Concurrency Model

The TFT is a shared hardware resource.

The current architecture effectively has:

```text
                ┌──────────────────────┐
                │     TFT / SPI1       │
                └──────────▲───────────┘
                           │
                    ScreenManager mutex
                           │
             ┌─────────────┴─────────────┐
             │                           │
       Task_Display                Task_Button_UI
             │                           │
       25 Hz update                  button event
```

The rule is:

> Every screen-manager-driven TFT access must be serialized.

This is why `ScreenManager_Lock()` and `ScreenManager_Unlock()` are exposed to code such as `ScreenGauge_SetStopped()` and `ScreenBoot_AddLog()` that may perform direct drawing from another task.

---

# 34. What the UI Task Does Not Own

The UI should not become the owner of control-system state.

For example, UI code should not directly implement:

```text
PID
trajectory generation
inverse kinematics
servo safety limits
CAN heartbeat supervision
IWDG handling
IMU fusion
```

Instead:

```text
UI
  │
  ├── reads display state
  └── sends user intent
```

while:

```text
Control/state/safety layers
  │
  ├── validate
  ├── apply
  └── execute
```

This separation is particularly important for safety-critical actions such as STOP and mode changes.

---

# 35. Current UI Data Flow

The complete data path is:

```text
                 Jetson
                   │
                  CAN
                   │
                   ▼
              Task_CAN_RX
                   │
                   ▼
            system_state
                   │
        ┌──────────┼───────────┐
        │          │           │
       ball       mode       camera
        │          │           │
        └──────────┼───────────┘
                   │
                   ▼
       UiData_SyncFromSystemState()
                   │
                   ▼
                g_uiData
                   │
                   ▼
          Task_Display @ 25 Hz
                   │
                   ▼
           ScreenManager
                   │
                   ▼
        screen_gauge_common
                   │
                   ▼
             TFT Service
                   │
                   ▼
               SPI1 DMA
                   │
                   ▼
                ILI9225
```

The reverse path for operator commands is:

```text
Physical buttons
      │
      ▼
Task_Button_UI
      │
      ▼
ButtonState_t
      │
      ▼
ScreenManager_OnButton()
      │
      ▼
active screen
      │
      ├── UI navigation
      ├── setpoint_set()
      └── StateRequestQueue
```

---

# 36. Performance Strategy

The display architecture uses several layers of optimization.

## Layer 1 — Fixed display rate

```text
25 Hz
```

The TFT does not need to update at the control-loop rate.

## Layer 2 — Static/dynamic separation

Static graphics are drawn once.

## Layer 3 — Dirty field update

Only changed values are redrawn.

## Layer 4 — Dirty ball update

Only the old/new ball region is modified.

## Layer 5 — Fast primitives

Use:

```text
TFT_FillCircleFast()
TFT_DrawTextFast()
```

for frequently updated regions.

## Layer 6 — DMA

Large pixel transfers use SPI DMA.

## Layer 7 — Cache maintenance

D-Cache is cleaned before DMA reads RAM buffers.

Together:

```text
25 Hz
  +
dirty rendering
  +
DMA
  +
cache coherency
  +
mutex serialization
```

gives a much more stable UI than simply redrawing the entire screen every frame.

---

# 37. Important Bugs Solved During UI Development

## 37.1 Mixed screens during navigation

### Symptom

Old gauge content and Shutdown content appeared simultaneously.

### Root cause

Two tasks were drawing to the TFT concurrently.

### Fix

Recursive ScreenManager mutex around screen operations.

---

## 37.2 TFT stopped responding after navigation

### Symptom

After entering another screen, the TFT could stop updating and buttons could appear dead.

### Root cause

A normal mutex could be acquired twice by the same task through nested screen-manager calls.

### Fix

Use a recursive CMSIS-RTOS2 mutex.

---

## 37.3 Button service starved by display rendering

### Symptom

The display task could consume CPU while waiting for SPI DMA.

### Root cause

Busy waiting / yielding did not necessarily remove the display task from the Ready state.

### Fix

Block on an RTOS semaphore and release it from the DMA-completion ISR path.

---

## 37.4 Text rendered mirrored

### Symptom

Solid graphics looked correct but text was reversed.

### Root cause

ILI9225 GRAM ordering combined with the selected logical rotation.

### Fix

Transform the text sprite before the DMA transfer.

---

## 37.5 DMA sent stale text/pixel data

### Symptom

Display data could be incorrect even though the CPU had written the buffer.

### Root cause

Cortex-M7 D-Cache and DMA coherency.

### Fix

`SCB_CleanDCache_by_Addr()` before DMA transmission.

---

## 37.6 Ball erased static geometry

### Symptom

Moving the ball could leave gaps in the crosshair or hexagon.

### Root cause

The old ball was erased using the background color.

### Fix

`RepairStaticNear()` restores only the geometry affected by the old ball.

---

# 38. Design Decisions Worth Explaining in an Interview

### Why not redraw the whole screen at 25 Hz?

Because the ILI9225 is relatively slow and the UI contains mostly static content. Redrawing everything creates unnecessary SPI traffic and visible flicker.

### Why use DMA for TFT?

Because the TFT transfer is mostly memory-to-peripheral data movement. DMA lets the CPU perform other work while SPI transfers pixels.

### Why use a semaphore instead of polling DMA completion?

Because polling keeps the task runnable and can interfere with lower-priority RTOS services. A semaphore blocks the task and wakes it immediately when DMA completes.

### Why use a recursive mutex?

Because screen callbacks can invoke screen-manager functions recursively in the same task. A normal mutex would self-deadlock.

### Why is UI state copied into `g_uiData`?

It creates a display-oriented boundary between the system-state/control architecture and the rendering code.

### Why use a screen stack?

Because faults and dialogs can temporarily interrupt another screen. A stack preserves the complete navigation history instead of only one previous screen.

### Why does STOP use an overlay?

Because the operator still benefits from seeing the diagnostic values while the robot is stopped.

---

# 39. Current Limitations / Technical Debt

The following items are visible in the current implementation and should not be presented as completed features.

## 39.1 Shutdown execution is not yet fully connected

`screen_shutdown.c` currently contains a TODO for invoking the actual safety/actuator shutdown procedure.

The UI therefore confirms the user's request but does not itself implement the complete physical shutdown sequence.

## 39.2 Fault reason is generic

The current Fault screen shows:

```text
FAULT
Check CAN and sensor signal
```

There is no detailed fault-code/message pipeline yet.

## 39.3 Shutdown log access should follow the same synchronization rule

`ScreenBoot_AddLog()` explicitly uses the ScreenManager mutex.

`ScreenShutdown_AddLog()` currently writes directly to the TFT without the same explicit locking layer.

If shutdown logs can be produced from a task other than the task currently executing the screen, this should be corrected so that all cross-task TFT writes follow one consistent synchronization rule.

## 39.4 Legacy Stop screen remains in the tree

`screen_stop.c` still exists, although the active emergency-stop design in the gauge architecture uses `ScreenGauge_SetStopped()` and an overlay.

The legacy screen should either be documented as historical code or removed once no references remain.

## 39.5 `guideText` still has a default legacy message

`UiData_Init()` currently initializes:

```text
Hold RED to STOP
```

while the current navigation architecture treats the long-press action separately from normal screen drawing.

If the final UI design intentionally removes this instruction, the default text should be changed/removed rather than leaving a stale operator instruction in the firmware.

---

# 40. Recommended Future Evolution

The current architecture is already strong enough for a portfolio project. Future improvements should preserve the same boundaries.

Recommended order:

### Phase 1 — finish current implementation

- Connect Shutdown screen to the real safety/actuator shutdown request.
- Add structured fault codes.
- Remove or clearly mark legacy Stop screen.
- Remove stale UI instructions.

### Phase 2 — improve display robustness

- Centralize all direct TFT writes behind one display/UI synchronization API.
- Add explicit UI state validation.
- Add optional display-transfer statistics for debugging.

### Phase 3 — improve visual design

- Replace the simple 5×7 font if memory/flash budget permits.
- Add consistent status icons.
- Add explicit units:
  - degrees for Roll/Pitch;
  - mm for ball position;
  - mm/s for ball velocity;
  - µs for servo commands.
- Add a compact fault-code display.

### Phase 4 — performance measurement

Measure:

```text
ScreenManager_Update() execution time
SPI DMA transfer time
25 Hz deadline margin
maximum mutex hold time
```

The goal is to verify that the display remains a non-real-time diagnostic task and never interferes with the control loop.

---

# 41. Final Architecture Summary

The current TFT UI architecture can be summarized as:

```text
                     ┌───────────────────────┐
                     │      User buttons     │
                     └───────────┬───────────┘
                                 │
                                 ▼
                       ┌──────────────────┐
                       │ Task_Button_UI   │
                       └────────┬─────────┘
                                │
                                ▼
                       ButtonState_t
                                │
                                ▼
                     ┌─────────────────────┐
                     │   ScreenManager     │
                     │ recursive mutex     │
                     │ navigation stack    │
                     └─────────┬───────────┘
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
          Gauge screens      Boot             Fault
              │             Shutdown
              │
              ▼
       screen_gauge_common
              │
              │ reads
              ▼
          g_uiData
              ▲
              │
              │
     UiData_SyncFromSystemState()
              ▲
              │
        system_state
              ▲
       ┌──────┴──────┐
       │             │
     CAN RX        Control/
     IMU/etc.      actuator

              ScreenManager
                    │
                    ▼
             TFT graphics API
                    │
            ┌───────┴────────┐
            │                │
        fast text          shapes
        sprite DMA       dirty drawing
            │                │
            └───────┬────────┘
                    ▼
                 SPI1 DMA
                    │
                    ▼
                  ILI9225
```

The central design principle is:

> **The TFT is a presentation device, not the owner of control state.**

The UI receives a synchronized display snapshot, converts user actions into validated application intent, and renders only what is necessary. Screen navigation is isolated in `ScreenManager`, shared display data is isolated in `g_uiData`, and hardware-specific rendering/performance details remain inside `tft_service`.

This separation is what makes the UI architecture suitable for an embedded real-time system rather than treating the TFT as a collection of ad-hoc drawing calls.

---

## 42. Source Files

Primary implementation files:

```text
firmware-stm32h723/App/bsp/tft_service.c
firmware-stm32h723/App/bsp/tft_service.h

firmware-stm32h723/App/UI/screen.h
firmware-stm32h723/App/UI/screen_manager.c
firmware-stm32h723/App/UI/screen_gauge_common.c
firmware-stm32h723/App/UI/screen_gauge_common.h
firmware-stm32h723/App/UI/screen_home.c
firmware-stm32h723/App/UI/screen_calibrate.c
firmware-stm32h723/App/UI/screen_balance.c
firmware-stm32h723/App/UI/screen_position.c
firmware-stm32h723/App/UI/screen_manual.c
firmware-stm32h723/App/UI/screen_boot.c
firmware-stm32h723/App/UI/screen_shutdown.c
firmware-stm32h723/App/UI/screen_fault.c
firmware-stm32h723/App/UI/screen_stop.c
firmware-stm32h723/App/UI/ui_data.c
firmware-stm32h723/App/UI/ui_data.h

firmware-stm32h723/App/tasks/task_display.c
firmware-stm32h723/App/tasks/task_button_ui.c
```

## 43. Engineering Design Summary

The TFT UI subsystem is designed as a non-real-time presentation layer within the STM32H723 control system.

Its architecture separates three responsibilities:

```text
Application / System State
        │
        ▼
     UI Data
        │
        ▼
   ScreenManager
        │
        ▼
  Screen Rendering
        │
        ▼
   TFT Service
        │
        ▼
   SPI1 + DMA
        │
        ▼
      ILI9225
```

Several design decisions are specifically intended to prevent the display subsystem from interfering with the real-time control system:

- **25 Hz display update** instead of running the UI at the control-loop frequency.
- **Dirty rendering** to avoid redrawing unchanged information.
- **DMA-based SPI transfers** for larger pixel buffers.
- **D-Cache maintenance** before DMA transmission on the Cortex-M7.
- **Semaphore-based DMA completion** instead of continuously polling the transfer state.
- **Recursive mutex protection** for ScreenManager and shared TFT access.
- **Screen stack** for temporary screens such as Fault and Shutdown.
- **Shared UI data snapshot** so rendering code does not become the owner of control-system state.

The resulting architecture keeps the responsibilities separated:

```text
Control / Safety
      │
      │ owns robot state
      ▼
   System State
      │
      │ snapshot
      ▼
    UI Data
      │
      │ presentation
      ▼
 ScreenManager
      │
      ▼
 TFT Service
```

This separation is important because the TFT is not part of the real-time control path. A display update should never become a dependency of the servo control loop, CAN processing, IMU processing, or safety mechanisms.

The main engineering considerations in this subsystem were therefore not only graphical rendering, but also **DMA/cache coherency, RTOS synchronization, execution timing, shared-resource protection, and minimizing display-side CPU/SPI overhead**.

---


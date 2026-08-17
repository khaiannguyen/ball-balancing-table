#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H
#include <stdint.h>
#include <stdbool.h>
#include "seqlock.h"
#include "task_state_machine.h"   // system_state_t

/* ---------- 0. Operating mode (THÊM cho B6, mục 12 THIẾT KẾ MODE VẬN HÀNH) ----------
 * Đây là "mode" mà UI chọn (0-4, hiển thị trên TFT) và Task_ControlLoop dùng
 * để dispatch (switch) sang đúng file control_mode_*.c khi system_state ==
 * STATE_RUN. Nguồn sự thật DUY NHẤT của mode hiện tại luôn là
 * setpoint_get().mode (mutex-protected) - không lưu mode ở đâu khác để
 * tránh lệch dữ liệu giữa UI và control loop.
 *
 * LƯU Ý ĐẶT TÊN: screen_gauge_common.h (tầng UI) đã có sẵn GaugeMode_t với
 * đúng các tên MODE_HOME/MODE_CALIBRATE/MODE_BALANCE/MODE_POSITION (giá trị
 * 0-3). Enum ở đây (tầng Control) dùng CHUNG GIÁ TRỊ SỐ với GaugeMode_t
 * (bắt buộc, vì cùng ghi vào setpoint.mode) nhưng phải đặt TIỀN TỐ KHÁC
 * (OPMODE_*) để không bị "redeclaration of enumerator" khi 1 file include
 * cả 2 header (ví dụ task_button_ui.c). Task_ControlLoop và các file
 * control_mode_*.c dùng OPMODE_*, tầng UI (screen_*.c) tiếp tục dùng
 * GaugeMode_t như cũ - 2 enum song song, cùng giá trị số, khác namespace. */
typedef enum {
    OPMODE_HOME     = 0,   // servo về S1/S2/S3 neutral, xong tự RUN->READY
    OPMODE_CALIB    = 1,   // P-controller tự động hội tụ a/b (mục 12.1)
    OPMODE_BALANCE  = 2,   // cascade Roll/Pitch/Height, mục 12.2
    OPMODE_POSITION = 3,   // giống Balance nhưng Ballx_d/Bally_d do UI chọn, mục 12.3
    OPMODE_MANUAL   = 4,   // test tay servo: deadband scan / chỉnh a-b tay / sweep log CSV
    OPMODE_COUNT_TOTAL      // = 5, dùng để validate giá trị mode hợp lệ khi cần
} operating_mode_t;

/* ---------- 1. IMU state — seqlock, writer duy nhất: Task_IMU_Fusion (mục 3.2) ---------- */
typedef struct {
    seqlock_t lock;
    float roll, pitch, vroll, vpitch;
} imu_state_t;

void imu_state_write(imu_state_t *s, float roll, float pitch, float vroll, float vpitch);
void imu_state_read(const imu_state_t *s, float *roll, float *pitch, float *vroll, float *vpitch);
imu_state_t *system_state_get_imu_ptr(void);   /* getter con trỏ, cùng pattern với imu_raw/actuator - dùng cho Task_CAN_TX */

/* ---------- 1b. IMU RAW state (trước filter) — seqlock, writer duy nhất: HAL_SPI_RxCpltCallback (ISR) ---------- */
typedef struct {
    seqlock_t lock;
    int16_t accel[3];
    int16_t gyro[3];
} imu_raw_state_t;

void imu_raw_state_write(imu_raw_state_t *s, const int16_t *accel, const int16_t *gyro);
void imu_raw_state_read(const imu_raw_state_t *s, int16_t *accel, int16_t *gyro);
imu_raw_state_t *system_state_get_imu_raw_ptr(void);

/* ---------- 1c. Ball state (nhận từ Jetson qua CAN 0x200/0x201/0x202, mục 4.3) —
   seqlock, writer duy nhất: Task_CAN_RX. Đọc bởi Task_ControlLoop (mode Balance/Position,
   mục 12.2/12.3) và Task_Display. ---------- */
typedef struct {
    seqlock_t lock;
    int16_t Ballx, Bally;        // mm, từ 0x200 BALL_POS
    int16_t vBallx, vBally;      // mm/s, từ 0x201 BALL_VEL
    uint8_t  ball_detected;      // 0/1, từ 0x202 BALL_STATE
} ball_state_t;

void ball_state_write_pos(ball_state_t *s, int16_t x, int16_t y);
void ball_state_write_vel(ball_state_t *s, int16_t vx, int16_t vy);
void ball_state_write_detected(ball_state_t *s, uint8_t detected);
void ball_state_read(const ball_state_t *s, int16_t *x, int16_t *y,
                      int16_t *vx, int16_t *vy, uint8_t *detected);
ball_state_t *system_state_get_ball_ptr(void);   // getter con trỏ, cùng pattern với imu_raw/actuator

/* ---------- 1d. Camera heartbeat (THÊM cho B5->B6) ----------
 * "cameraOk" KHÁC "ball_detected" (0x202): cameraOk = Jetson còn
 * sống/gửi dữ liệu, ball_detected = Jetson thấy bóng hay không.
 * Nguồn sự thật: Task_CAN_RX gọi camera_heartbeat_mark() mỗi khi
 * parse xong 0x200 BALL_POS hoặc 0x201 BALL_VEL (2 frame Jetson
 * gửi định kỳ khi còn sống). Task_Display/UI đọc qua
 * camera_state_is_ok() = "có nhận được 1 trong 2 frame đó trong
 * 500ms gần nhất không" - cùng ngưỡng với HEARTBEAT_RX failsafo
 * chung (mục 4.2 CAN table, 0x2FF, 500ms). Không cần seqlock vì
 * chỉ 1 biến tick, single-writer (Task_CAN_RX) / multi-reader. */
void camera_heartbeat_mark(void);
bool camera_state_is_ok(void);

/* ---------- 2. Actuator snapshot — seqlock, writer duy nhất: Task_ControlLoop (mục 11.2) ---------- */
typedef struct {
    seqlock_t lock;
    int32_t S1, S2, S3;   // giá trị PWM us đã clamp/slew, dùng để publish ra ngoài
} actuator_state_t;

void actuator_state_publish(actuator_state_t *s, int32_t s1, int32_t s2, int32_t s3);
void actuator_state_read(const actuator_state_t *s, int32_t *s1, int32_t *s2, int32_t *s3);
actuator_state_t *system_state_get_actuator_ptr(void);   // getter con trỏ, cùng pattern với imu_raw

/* ---------- 3. Setpoint — Mutex, tần số thấp (mục 3.2) ---------- */
typedef struct {
    uint8_t mode;                              // giá trị của operating_mode_t (mục 0, THÊM B6) -
                                                // 0=OPMODE_HOME,1=OPMODE_CALIB,2=OPMODE_BALANCE,
                                                // 3=OPMODE_POSITION,4=OPMODE_MANUAL - CÙNG giá trị
                                                // số với GaugeMode_t bên UI (screen_gauge_common.h)
    float   Roll_d, Pitch_d, Height_d;         // từ CAN 0x204
    float   Ballx_d, Bally_d;                  // từ Task_Button_UI (mục 12.3)
} setpoint_t;

bool setpoint_get(setpoint_t *out);            // false nếu không lấy được mutex kịp
bool setpoint_set(const setpoint_t *in);

/* ---------- 4. System state hiện tại — seqlock, writer duy nhất: Task_StateMachine (mục 14.2) ---------- */
void system_state_publish(system_state_t s);
system_state_t system_state_get(void);

/* ---------- 5. Task-level watchdog alive mask (mục 3.6) ---------- */
#define ALIVE_BIT_CONTROL_LOOP   (1u << 0)
#define ALIVE_BIT_IMU_FUSION     (1u << 1)
#define ALIVE_BIT_CAN_RX         (1u << 2)
#define ALIVE_MASK_EXPECTED      (ALIVE_BIT_CONTROL_LOOP | ALIVE_BIT_IMU_FUSION | ALIVE_BIT_CAN_RX)

void     task_alive_mark(uint32_t bit);
uint32_t task_alive_snapshot_and_clear(void);

/* ---------- 6. Event Group bits (mục 3.2 dòng "Cờ trạng thái nhiều bit") ---------- */
#define EVT_BIT_RUN            (1u << 0)
#define EVT_BIT_STOP           (1u << 1)
#define EVT_BIT_BALL_DETECTED  (1u << 2)
#define EVT_BIT_CALIBRATED     (1u << 3)
#define EVT_BIT_FAULT          (1u << 4)

/* THÊM cho B7: Task_ControlLoop set bit này ngay sau khi IMU init + calib
 * load/apply + servo_actuator_init() xong - Task_Display chờ bit này
 * (osEventFlagsWait, clear-on-read mặc định) thay vì osDelay(2000) cứng
 * trước khi rời màn Boot. Dùng chung SystemEventGroupHandle đã có sẵn
 * (không tạo Event Group mới trong CubeMX). */
#define EVT_BIT_BOOT_DONE      (1u << 5)

/* THÊM cho B7 (fix deadlock màn Boot): Task_Display set bit này NGAY SAU
 * TFT_Init() + ScreenManager_Goto(ScreenBoot_Get()) - tức là sau khi khung
 * Boot đã thực sự được vẽ. Task_ControlLoop PHẢI chờ bit này trước khi gọi
 * ScreenBoot_AddLog() lần đầu tiên, nếu không TFT_DrawText() bên trong sẽ
 * chạy khi phần cứng TFT/DMA CHƯA init xong -> có thể treo vô hạn (chờ
 * semaphore DMA không bao giờ hoàn tất) trong lúc đang giữ ScreenManager
 * mutex -> Task_Display gọi ScreenManager_Goto(Boot) sau đó cũng cần đúng
 * mutex đó -> deadlock vĩnh viễn, màn hình trắng không hiện gì (đã gặp
 * thực tế). */
#define EVT_BIT_TFT_READY      (1u << 6)

void system_state_init(void);   // gọi 1 lần trong StartDefaultTask/app init, sau khi kernel start

/* ---------- 7. Fault peek (THÊM cho B5) ----------
 * Dùng chung 1 pattern với event_flag_is_set() tĩnh trong task_can_tx.c
 * (osFlagsNoClear, timeout 0) - expose ra đây để Task_Display và
 * Task_Button_UI (App/UI) đọc mà không cần tự viết lại, và không
 * tranh chấp với Task_ControlLoop/Task_CAN_TX cũng đang đọc cùng bit. */
bool system_state_fault_is_set(void);

#endif

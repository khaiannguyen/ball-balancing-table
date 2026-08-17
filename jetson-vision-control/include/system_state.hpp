#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP
#include <cstdint>
#include "seqlock.hpp"

/* ---------- 1. Telemetry nhận từ STM32 (0x100 ATTITUDE + 0x101 RATE) ---------- */
struct telemetry_attitude_t {
    seqlock_t lock;
    float roll = 0, pitch = 0;
    float vroll = 0, vpitch = 0;
    float height = 0;
};

void telemetry_attitude_write(telemetry_attitude_t &s, float roll, float pitch,
                               float vroll, float vpitch, float height);
void telemetry_attitude_read(const telemetry_attitude_t &s, float &roll, float &pitch,
                              float &vroll, float &vpitch, float &height);

/* ---------- 2. Servo pos nhận từ STM32 (0x102 SERVO_POS) ---------- */
struct telemetry_servo_t {
    seqlock_t lock;
    int32_t S1 = 0, S2 = 0, S3 = 0;
};

void telemetry_servo_write(telemetry_servo_t &s, int32_t s1, int32_t s2, int32_t s3);
void telemetry_servo_read(const telemetry_servo_t &s, int32_t &s1, int32_t &s2, int32_t &s3);

/* ---------- 3. Robot state nhận từ STM32 (0x103 ROBOT_STATE) ---------- */
struct telemetry_robot_state_t {
    seqlock_t lock;
    uint8_t mode = 0;
    uint8_t bits = 0;
};

void telemetry_robot_state_write(telemetry_robot_state_t &s, uint8_t mode, uint8_t bits);
void telemetry_robot_state_read(const telemetry_robot_state_t &s, uint8_t &mode, uint8_t &bits);

/* ---------- 4. Ball state — Jetson GHI, gửi đi qua 0x200/0x201/0x202 ---------- */
struct ball_state_t {
    seqlock_t lock;
    int16_t Ballx = 0, Bally = 0;
    int16_t vBallx = 0, vBally = 0;
    uint8_t ball_detected = 0;
};

void ball_state_write_pos(ball_state_t &s, int16_t x, int16_t y);
void ball_state_write_vel(ball_state_t &s, int16_t vx, int16_t vy);
void ball_state_write_detected(ball_state_t &s, uint8_t detected);
void ball_state_read(const ball_state_t &s, int16_t &x, int16_t &y,
                      int16_t &vx, int16_t &vy, uint8_t &detected);

/* ---------- 4b. Ball desired — Jetson NHẬN từ STM32 qua 0x104 BALL_DESIRED
 * (Ballx_d, Bally_d, Ballheight_d, int16 mm). Writer duy nhất: TaskCanRx.
 * Đọc bởi tầng logic Jetson khi cần biết STM32 đang muốn bóng ở vị trí nào
 * (vd Mode Position, mục 12.3) - đối xứng với setpoint_t.Ballx_d/Bally_d
 * bên STM32 (system_state.h), chỉ khác chiều truyền. ---------- */
struct ball_desired_t {
    seqlock_t lock;
    int16_t Ballx_d = 0, Bally_d = 0;
    int16_t Ballheight_d = 0;
};

void ball_desired_write(ball_desired_t &s, int16_t x_d, int16_t y_d, int16_t height_d);
void ball_desired_read(const ball_desired_t &s, int16_t &x_d, int16_t &y_d, int16_t &height_d);

/* ---------- 5. Attitude desired — Jetson GHI (output PID, J7), gửi đi
 * qua 0x204 ATTITUDE_DESIRED. Đơn vị: roll_d/pitch_d = ĐỘ (float),
 * TaskCanTx chịu trách nhiệm nhân 100 khi đóng gói i16le, đối xứng với
 * cách task_can_rx.cpp đọc 0x100 (chia 100.0f khi giải mã).
 *
 * THÊM (Kế hoạch 2 - Height Control): height_d = MM (float, KHÔNG nhân
 * 100 - khớp đúng cách STM32 task_can_rx.c đang giải mã byte 4-5 của
 * 0x204: "can_rd_i16le(&frame.data[4])" đọc thẳng làm mm, không chia
 * 100). TaskCanTx phía Jetson khi đóng gói byte 4-5 PHẢI ép tròn thẳng
 * sang int16_t (không nhân 100) - xem TODO ở task_can_tx.cpp, file này
 * hiện chưa thấy nên CHƯA sửa được, cần bạn gửi thêm để xác nhận/sửa. */
struct attitude_desired_t {
    seqlock_t lock;
    float roll_d = 0.f;
    float pitch_d = 0.f;
    float height_d = 0.f;
};

void attitude_desired_write(attitude_desired_t &s, float roll_d, float pitch_d, float height_d);
void attitude_desired_read(const attitude_desired_t &s, float &roll_d, float &pitch_d, float &height_d);

/* ---------- 6. STM32-connection heartbeat ---------- */
void stm32_heartbeat_mark();
bool stm32_state_is_ok();

/* ---------- Singleton accessor ---------- */
struct SystemState {
    telemetry_attitude_t     attitude;
    telemetry_servo_t        servo;
    telemetry_robot_state_t  robot_state;
    ball_state_t              ball;
    ball_desired_t             ball_desired;
    attitude_desired_t        attitude_desired;
};

SystemState &system_state();

#endif
#include "system_state.hpp"
#include <ctime>

void telemetry_attitude_write(telemetry_attitude_t &s, float roll, float pitch,
                               float vroll, float vpitch, float height) {
    seqlock_write_begin(s.lock);
    s.roll = roll; s.pitch = pitch;
    s.vroll = vroll; s.vpitch = vpitch;
    s.height = height;
    seqlock_write_end(s.lock);
}
void telemetry_attitude_read(const telemetry_attitude_t &s, float &roll, float &pitch,
                              float &vroll, float &vpitch, float &height) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        roll = s.roll; pitch = s.pitch;
        vroll = s.vroll; vpitch = s.vpitch;
        height = s.height;
    } while (seqlock_read_retry(s.lock, start));
}

void telemetry_servo_write(telemetry_servo_t &s, int32_t s1, int32_t s2, int32_t s3) {
    seqlock_write_begin(s.lock);
    s.S1 = s1; s.S2 = s2; s.S3 = s3;
    seqlock_write_end(s.lock);
}
void telemetry_servo_read(const telemetry_servo_t &s, int32_t &s1, int32_t &s2, int32_t &s3) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        s1 = s.S1; s2 = s.S2; s3 = s.S3;
    } while (seqlock_read_retry(s.lock, start));
}

void telemetry_robot_state_write(telemetry_robot_state_t &s, uint8_t mode, uint8_t bits) {
    seqlock_write_begin(s.lock);
    s.mode = mode; s.bits = bits;
    seqlock_write_end(s.lock);
}
void telemetry_robot_state_read(const telemetry_robot_state_t &s, uint8_t &mode, uint8_t &bits) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        mode = s.mode; bits = s.bits;
    } while (seqlock_read_retry(s.lock, start));
}

void ball_state_write_pos(ball_state_t &s, int16_t x, int16_t y) {
    seqlock_write_begin(s.lock);
    s.Ballx = x; s.Bally = y;
    seqlock_write_end(s.lock);
}
void ball_state_write_vel(ball_state_t &s, int16_t vx, int16_t vy) {
    seqlock_write_begin(s.lock);
    s.vBallx = vx; s.vBally = vy;
    seqlock_write_end(s.lock);
}
void ball_state_write_detected(ball_state_t &s, uint8_t detected) {
    seqlock_write_begin(s.lock);
    s.ball_detected = detected;
    seqlock_write_end(s.lock);
}
void ball_state_read(const ball_state_t &s, int16_t &x, int16_t &y,
                      int16_t &vx, int16_t &vy, uint8_t &detected) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        x = s.Ballx; y = s.Bally;
        vx = s.vBallx; vy = s.vBally;
        detected = s.ball_detected;
    } while (seqlock_read_retry(s.lock, start));
}
void ball_desired_write(ball_desired_t &s, int16_t x_d, int16_t y_d, int16_t height_d) {
    seqlock_write_begin(s.lock);
    s.Ballx_d = x_d; s.Bally_d = y_d; s.Ballheight_d = height_d;
    seqlock_write_end(s.lock);
}
void ball_desired_read(const ball_desired_t &s, int16_t &x_d, int16_t &y_d, int16_t &height_d) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        x_d = s.Ballx_d; y_d = s.Bally_d; height_d = s.Ballheight_d;
    } while (seqlock_read_retry(s.lock, start));
}

/* SUA (Ke hoach 2): them height_d (mm, float) - ghi/doc them field moi
 * attitude_desired_t::height_d, cung co che seqlock nhu roll_d/pitch_d,
 * KHONG doi gi ve khoa/dong bo. */
void attitude_desired_write(attitude_desired_t &s, float roll_d, float pitch_d, float height_d) {
    seqlock_write_begin(s.lock);
    s.roll_d = roll_d; s.pitch_d = pitch_d; s.height_d = height_d;
    seqlock_write_end(s.lock);
}
void attitude_desired_read(const attitude_desired_t &s, float &roll_d, float &pitch_d, float &height_d) {
    uint32_t start;
    do {
        start = seqlock_read_begin(s.lock);
        roll_d = s.roll_d; pitch_d = s.pitch_d; height_d = s.height_d;
    } while (seqlock_read_retry(s.lock, start));
}

namespace {
    std::atomic<int64_t> g_stm32_last_rx_ms{0};
    constexpr int64_t STM32_HEARTBEAT_TIMEOUT_MS = 500;

    int64_t now_ms() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    }
}

void stm32_heartbeat_mark() {
    g_stm32_last_rx_ms.store(now_ms(), std::memory_order_relaxed);
}
bool stm32_state_is_ok() {
    int64_t last = g_stm32_last_rx_ms.load(std::memory_order_relaxed);
    return (now_ms() - last) < STM32_HEARTBEAT_TIMEOUT_MS;
}

SystemState &system_state() {
    static SystemState instance;
    return instance;
}
#include "task_can_tx.hpp"
#include "task_watchdog.hpp"
#include "system_state.hpp"
#include "can_protocol.h"
#include <cstdio>
#include <chrono>
#include <cmath>
#include <pthread.h>

bool TaskCanTx::start(const std::string &ifname) {
    if (running_.load()) return true;

    if (!can_.open(ifname)) {
        std::printf("TaskCanTx: khong mo duoc %s\n", ifname.c_str());
        return false;
    }

    running_.store(true);
    thread_ = std::make_unique<std::thread>(&TaskCanTx::run, this);
    return true;
}

void TaskCanTx::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (thread_ && thread_->joinable()) thread_->join();
    can_.close();
}

TaskCanTx::~TaskCanTx() {
    stop();
}

void TaskCanTx::run() {
    std::printf("TaskCanTx: STARTED\n");

    // THEM: tuong tu TaskCanRx — nang priority de tranh bi TaskControlLoop
    // (SCHED_FIFO 80) hoac TaskBallDetect chiem CPU lam tre viec gui
    // 0x204 (roll_d/pitch_d) toi STM32, nguy hiem cho vong dieu khien
    // that neu STM32 nhan lenh tre. Priority = 70, THAP HON control loop
    // (giu nguyen tac uu tien an toan cao nhat cho vong dieu khien),
    // NGANG BANG TaskCanRx (ca 2 cung quan trong nhu nhau ve tinh kip
    // thoi cua kenh CAN, khong can phan biet hon kem giua RX/TX).
    struct sched_param sp;
    sp.sched_priority = 70;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "TaskCanTx: khong set duoc SCHED_FIFO "
                     "(can CAP_SYS_NICE/sudo), chay priority mac dinh - "
                     "co the bi doi CPU boi TaskControlLoop/TaskBallDetect\n");
    }

    using clock = std::chrono::steady_clock;
    auto next_wake = clock::now();
    const auto period = std::chrono::milliseconds(10);   // 100Hz

    uint8_t heartbeat_counter = 0;

    while (running_.load()) {
        next_wake += period;

        uint8_t buf[8];

        /* ---- Doc ball state (ghi boi TaskBallDetect, J6) de gui di ---- */
        int16_t x, y, vx, vy;
        uint8_t detected;
        ball_state_read(system_state().ball, x, y, vx, vy, detected);

        can_wr_i16le(&buf[0], x);
        can_wr_i16le(&buf[2], y);
        can_.send(CAN_ID_BALL_POS, 4, buf);                     /* 0x200 */

        can_wr_i16le(&buf[0], vx);
        can_wr_i16le(&buf[2], vy);
        can_.send(CAN_ID_BALL_VEL, 4, buf);                     /* 0x201 */

        buf[0] = detected;
        can_.send(CAN_ID_BALL_STATE, 1, buf);                   /* 0x202 */

        /* ---- Doc attitude_desired (ghi boi TaskControlLoop, J7) va gui
         * qua 0x204 ---- 6 byte (roll_d, pitch_d, height_d) de khop dung
         * dieu kien `frame.dlc >= 6` ben STM32 (task_can_rx.c).
         * SUA (Ke hoach 2): height_d gio DOC THAT tu attitude_desired (J7
         * dang gui co dinh 0.0f, profile tang bong that se lam sau - xem
         * task_control_loop.cpp), KHONG con hardcode 0 o day nua. */
        float roll_d, pitch_d, height_d;
        attitude_desired_read(system_state().attitude_desired, roll_d, pitch_d, height_d);

        int16_t roll_d_raw   = (int16_t)(roll_d  * 100.0f);
        int16_t pitch_d_raw  = (int16_t)(pitch_d * 100.0f);
        // height_d don vi MM (float) - KHONG nhan 100, khop dung cach
        // STM32 task_can_rx.c giai ma byte 4-5: can_rd_i16le(&frame.data[4])
        // doc THANG lam mm, khong chia 100 (khac roll/pitch nhan/chia 100).
        int16_t height_d_raw = (int16_t)std::lround(height_d);

        can_wr_i16le(&buf[0], roll_d_raw);
        can_wr_i16le(&buf[2], pitch_d_raw);
        can_wr_i16le(&buf[4], height_d_raw);
        can_.send(CAN_ID_ATTITUDE_DESIRED, 6, buf);              /* 0x204, DLC=6 */

        buf[0] = heartbeat_counter++;
        can_.send(CAN_ID_HEARTBEAT_RX, 1, buf);                 /* 0x2FF */


        std::this_thread::sleep_until(next_wake);
        task_alive_mark(ALIVE_BIT_CAN_TX);
    }

    std::printf("TaskCanTx: STOPPED\n");
}
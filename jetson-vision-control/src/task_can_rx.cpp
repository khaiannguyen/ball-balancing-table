#include "task_can_rx.hpp"
#include "task_watchdog.hpp"
#include "system_state.hpp"
#include "can_protocol.h"
#include <cstdio>
#include <chrono>
#include <pthread.h>

bool TaskCanRx::start(const std::string &ifname) {
    if (running_.load()) return true;

    if (!can_.open(ifname)) {
        std::printf("TaskCanRx: khong mo duoc %s\n", ifname.c_str());
        return false;
    }

    running_.store(true);
    thread_ = std::make_unique<std::thread>(&TaskCanRx::run, this);
    return true;
}

void TaskCanRx::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (thread_ && thread_->joinable()) thread_->join();
    can_.close();
}

TaskCanRx::~TaskCanRx() {
    stop();
}

void TaskCanRx::run() {
    std::printf("TaskCanRx: STARTED\n");

    // THEM: nang priority SCHED_FIFO cho TaskCanRx. Xac nhan qua log thuc
    // te (khong co bat ky loi poll()/read()/socket nao duoc ghi tu
    // can_transport.cpp, trong khi stm32_ok van roi ve 0 keo dai >500ms
    // du CAN bus van co du lieu that - xac nhan bang candump song song)
    // rang nguyen nhan KHONG phai loi tang socket, ma la THREAD NAY DON
    // GIAN KHONG DUOC CAP PHAT CPU DU LAU de goi poll(). TaskControlLoop
    // chay SCHED_FIFO priority 80 (real-time) co the chiem CPU lien tuc,
    // "doi" (starve) cac thread SCHED_OTHER binh thuong nhu thread nay,
    // dac biet neu Jetson it core va TaskBallDetect (OpenCV) cung dang
    // tieu ton CPU nang cung luc.
    //
    // Priority = 70, THAP HON TaskControlLoop (80) mot cach CO CHU DICH:
    // dam bao TaskCanRx van duoc uu tien hon cac thread thuong (khong bi
    // doi qua lau), nhung KHONG duoc phep chiem CPU truoc TaskControlLoop
    // — vi TaskControlLoop la vong dieu khien an toan realtime, uu tien
    // an toan phai luon cao nhat he thong.
    struct sched_param sp;
    sp.sched_priority = 70;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        std::fprintf(stderr, "TaskCanRx: khong set duoc SCHED_FIFO "
                     "(can CAP_SYS_NICE/sudo), chay priority mac dinh - "
                     "co the bi doi CPU boi TaskControlLoop/TaskBallDetect\n");
    }

    can_frame_t frame;

    while (running_.load()) {
        bool got = can_.receive(frame, 50);

        if (got) {
            switch (frame.id) {

            case CAN_ID_ATTITUDE:                                /* 0x100 */
                if (frame.dlc >= 6) {
                    float roll   = can_rd_i16le(&frame.data[0]) / 100.0f;
                    float pitch  = can_rd_i16le(&frame.data[2]) / 100.0f;
                    float height = (float)can_rd_i16le(&frame.data[4]);

                    float r, p, vr, vp, h;
                    telemetry_attitude_read(system_state().attitude, r, p, vr, vp, h);
                    telemetry_attitude_write(system_state().attitude, roll, pitch, vr, vp, height);

                    stm32_heartbeat_mark();
                }
                break;

            case CAN_ID_RATE:                                    /* 0x101 */
                if (frame.dlc >= 4) {
                    float vroll  = can_rd_i16le(&frame.data[0]) / 100.0f;
                    float vpitch = can_rd_i16le(&frame.data[2]) / 100.0f;

                    float r, p, vr, vp, h;
                    telemetry_attitude_read(system_state().attitude, r, p, vr, vp, h);
                    telemetry_attitude_write(system_state().attitude, r, p, vroll, vpitch, h);
                }
                break;

            case CAN_ID_SERVO_POS:                               /* 0x102 */
                if (frame.dlc >= 6) {
                    int32_t s1 = can_rd_u16le(&frame.data[0]);
                    int32_t s2 = can_rd_u16le(&frame.data[2]);
                    int32_t s3 = can_rd_u16le(&frame.data[4]);
                    telemetry_servo_write(system_state().servo, s1, s2, s3);
                    stm32_heartbeat_mark();
                }
                break;

            case CAN_ID_ROBOT_STATE:                             /* 0x103 */
                if (frame.dlc >= 2) {
                    telemetry_robot_state_write(system_state().robot_state,
                                                 frame.data[0], frame.data[1]);
                    stm32_heartbeat_mark();
                }
                break;

            case CAN_ID_BALL_DESIRED:                            /* 0x104 */
                if (frame.dlc >= 6) {
                    int16_t x_d      = can_rd_i16le(&frame.data[0]);
                    int16_t y_d      = can_rd_i16le(&frame.data[2]);
                    int16_t height_d = can_rd_i16le(&frame.data[4]);
                    ball_desired_write(system_state().ball_desired, x_d, y_d, height_d);
                    stm32_heartbeat_mark();
                }
                break;

            case CAN_ID_HEARTBEAT_TX:                            /* 0x1FF */
                break;

            default:
                break;
            }
        }

        static bool fault_latched = false;
        bool ok = stm32_state_is_ok();
        if (!ok && !fault_latched) {
            fault_latched = true;
            std::printf("TaskCanRx: FAULT - mat ket noi STM32!\n");
        } else if (ok && fault_latched) {
            fault_latched = false;
            std::printf("TaskCanRx: FAULT CLEARED - STM32 tro lai!\n");
        }
        task_alive_mark(ALIVE_BIT_CAN_RX);
    }

    std::printf("TaskCanRx: STOPPED\n");
}
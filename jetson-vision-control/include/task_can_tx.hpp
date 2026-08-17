#ifndef TASK_CAN_TX_HPP
#define TASK_CAN_TX_HPP
#include <thread>
#include <atomic>
#include <memory>
#include "can_transport.hpp"

/* Task_CAN_TX (Jetson) — gửi định kỳ 100Hz: 0x200 BALL_POS, 0x201
 * BALL_VEL, 0x202 BALL_STATE, 0x204 ATTITUDE_DESIRED (J7, PID output),
 * và 0x2FF HEARTBEAT_RX.
 *
 * J7: đã bỏ hẳn fake_mode của J4. Chỉ ĐỌC system_state().ball (ghi bởi
 * TaskBallDetect, J6) và system_state().attitude_desired (ghi bởi
 * TaskControlLoop, J7) — không tự ghi dữ liệu ball nữa. */
class TaskCanTx {
public:
    bool start(const std::string &ifname = "can0");
    void stop();
    ~TaskCanTx();

private:
    void run();

    std::atomic<bool> running_{false};
    CanTransport can_;
    std::unique_ptr<std::thread> thread_;
};

#endif
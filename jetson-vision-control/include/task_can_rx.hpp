#ifndef TASK_CAN_RX_HPP
#define TASK_CAN_RX_HPP
#include <thread>
#include <atomic>
#include <memory>
#include "can_transport.hpp"

/* Task_CAN_RX (Jetson, J3) — đối xứng với task_can_rx.c bên STM32:
 * - Nhận frame từ CanTransport, decode theo can_id.h/can_protocol.h
 * - Ghi vào system_state() (telemetry_attitude/servo/robot_state)
 * - Gọi stm32_heartbeat_mark() khi nhận 0x100/0x102/0x103 (frame STM32
 *   gửi định kỳ 100Hz) - dùng để phát hiện mất kết nối STM32, tương tự
 *   camera_heartbeat_mark() bên STM32 nhưng theo chiều ngược lại.
 *
 * Khác biệt so với STM32: không có FreeRTOS Queue/ISR, chạy bằng std::thread
 * với vòng lặp blocking-read có timeout (CanTransport::receive), y hệt cấu
 * trúc "for(;;) { xQueueReceive(timeout); ... failsafe check; }" bên STM32
 * nhưng thay Queue bằng socket read có timeout. */
class TaskCanRx {
public:
    bool start(const std::string &ifname = "can0");
    void stop();
    ~TaskCanRx();

private:
    void run();

    std::atomic<bool> running_{false};
    CanTransport can_;
    std::unique_ptr<std::thread> thread_;
};

#endif
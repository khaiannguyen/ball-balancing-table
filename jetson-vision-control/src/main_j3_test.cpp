#include "task_can_rx.hpp"
#include "system_state.hpp"
#include <cstdio>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true); }

int main() {
    std::signal(SIGINT, on_sigint);

    TaskCanRx can_rx;
    if (!can_rx.start("can0")) {
        std::printf("Khong start duoc TaskCanRx\n");
        return 1;
    }

    std::printf("TaskCanRx dang chay. Nhan Ctrl+C de dung.\n");

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        float roll, pitch, vroll, vpitch, height;
        telemetry_attitude_read(system_state().attitude, roll, pitch, vroll, vpitch, height);

        int32_t s1, s2, s3;
        telemetry_servo_read(system_state().servo, s1, s2, s3);

        uint8_t mode, bits;
        telemetry_robot_state_read(system_state().robot_state, mode, bits);

        std::printf("roll=%.2f pitch=%.2f vroll=%.2f vpitch=%.2f height=%.0f | "
                     "S1=%d S2=%d S3=%d | mode=%d bits=0x%02X | stm32_ok=%d\n",
                     roll, pitch, vroll, vpitch, height, s1, s2, s3, mode, bits,
                     stm32_state_is_ok());
    }

    can_rx.stop();
    std::printf("Da dung sach.\n");
    return 0;
}
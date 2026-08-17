#include "task_can_rx.hpp"
#include "task_can_tx.hpp"
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
    TaskCanTx can_tx;

    if (!can_rx.start("can0")) {
        std::printf("Khong start duoc TaskCanRx\n");
        return 1;
    }
    if (!can_tx.start("can0")) {
        std::printf("Khong start duoc TaskCanTx\n");
        can_rx.stop();
        return 1;
    }

    std::printf("TaskCanRx + TaskCanTx dang chay. Nhan Ctrl+C de dung.\n");

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        float roll, pitch, vroll, vpitch, height;
        telemetry_attitude_read(system_state().attitude, roll, pitch, vroll, vpitch, height);

        int16_t bx, by, bvx, bvy;
        uint8_t bdet;
        ball_state_read(system_state().ball, bx, by, bvx, bvy, bdet);

        std::printf("[RX] roll=%.2f pitch=%.2f | [TX] ball_x=%d ball_y=%d detected=%d | stm32_ok=%d\n",
                     roll, pitch, bx, by, bdet, stm32_state_is_ok());
    }

    can_tx.stop();
    can_rx.stop();
    std::printf("Da dung sach.\n");
    return 0;
}
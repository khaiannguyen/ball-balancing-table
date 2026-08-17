#include "task_watchdog.hpp"
#include <cstdio>
#include <chrono>

namespace {
    std::atomic<uint32_t> g_task_alive_mask{0};
}

void task_alive_mark(uint32_t bit) {
    g_task_alive_mask.fetch_or(bit, std::memory_order_relaxed);
}

uint32_t task_alive_snapshot_and_clear() {
    return g_task_alive_mask.exchange(0, std::memory_order_relaxed);
}

bool TaskWatchdog::start(uint32_t check_period_ms) {
    if (running_.load()) return true;
    running_.store(true);
    thread_ = std::make_unique<std::thread>(&TaskWatchdog::run, this, check_period_ms);
    std::printf("TaskWatchdog: da start (check moi %ums)\n", check_period_ms);
    return true;
}

void TaskWatchdog::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (thread_ && thread_->joinable()) thread_->join();
    std::printf("TaskWatchdog: da dung.\n");
}

void TaskWatchdog::run(uint32_t check_period_ms) {
    using clock = std::chrono::steady_clock;
    auto next_wake = clock::now();
    const auto period = std::chrono::milliseconds(check_period_ms);

    while (running_.load()) {
        next_wake += period;
        std::this_thread::sleep_until(next_wake);

        uint32_t snapshot = task_alive_snapshot_and_clear();
        uint32_t missing = ALIVE_MASK_EXPECTED & ~snapshot;

        if (missing != 0) {
            std::fprintf(stderr, "TaskWatchdog: CANH BAO - thread khong phan hoi, missing_mask=0x%02X (",
                         missing);
            if (missing & ALIVE_BIT_CAMERA)       std::fprintf(stderr, "Camera ");
            if (missing & ALIVE_BIT_BALL_DETECT)  std::fprintf(stderr, "BallDetect ");
            if (missing & ALIVE_BIT_CONTROL_LOOP) std::fprintf(stderr, "ControlLoop ");
            if (missing & ALIVE_BIT_CAN_RX)       std::fprintf(stderr, "CanRx ");
            if (missing & ALIVE_BIT_CAN_TX)       std::fprintf(stderr, "CanTx ");
            std::fprintf(stderr, ")\n");
            /* J8 pham vi: CHI phat hien + log. Auto-recovery tung thread
             * (vd tu restart TaskCameraCapture) de lai cho J9 neu can -
             * rui ro cao neu lam voi lai khong ky, de gay thread leak
             * hoac double-start. */
        }
    }
}
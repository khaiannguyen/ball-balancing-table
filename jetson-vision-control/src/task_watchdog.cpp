/**
 * @file    task_watchdog.cpp
 * @brief   Runtime task liveness supervision.
 *
 * Collects liveness indications from critical runtime tasks and periodically
 * checks whether all expected tasks have reported activity.
 *
 * The watchdog currently reports missing tasks only. Automatic task restart
 * is intentionally outside the watchdog scope to avoid unsafe thread
 * recreation and ownership problems.
 */

#include "task_watchdog.hpp"

#include <cstdio>
#include <chrono>

namespace
{

    std::atomic<uint32_t> g_task_alive_mask{ 0 };

}

/*
 * Set the liveness bit associated with a successfully executing task.
 *
 * Multiple tasks may update the mask concurrently, therefore the operation
 * is performed atomically without requiring a mutex.
 */
void task_alive_mark(uint32_t bit)
{
    g_task_alive_mask.fetch_or(
        bit,
        std::memory_order_relaxed
    );
}

/*
 * Atomically obtain the current liveness snapshot and clear it for the next
 * watchdog interval.
 *
 * Clearing during the snapshot operation creates a well-defined supervision
 * window: every expected task must report at least once during each interval.
 */
uint32_t task_alive_snapshot_and_clear()
{
    return g_task_alive_mask.exchange(
        0,
        std::memory_order_relaxed
    );
}

bool TaskWatchdog::start(
    uint32_t check_period_ms)
{
    if (running_.load())
    {
        return true;
    }

    running_.store(true);

    thread_ =
        std::make_unique<std::thread>(
            &TaskWatchdog::run,
            this,
            check_period_ms
            );

    std::printf(
        "TaskWatchdog: da start (check moi %ums)\n",
        check_period_ms
    );

    return true;
}

void TaskWatchdog::stop()
{
    if (!running_.load())
    {
        return;
    }

    running_.store(false);

    if (
        thread_ &&
        thread_->joinable())
    {
        thread_->join();
    }

    std::printf(
        "TaskWatchdog: da dung.\n"
    );
}

void TaskWatchdog::run(
    uint32_t check_period_ms)
{
    using clock =
        std::chrono::steady_clock;

    auto next_wake =
        clock::now();

    const auto period =
        std::chrono::milliseconds(
            check_period_ms
        );

    while (running_.load())
    {
        next_wake += period;

        std::this_thread::sleep_until(
            next_wake
        );

        /*
         * Capture and clear the liveness state at the end of each supervision
         * interval.
         *
         * A task is considered responsive when it has reported at least once
         * since the previous watchdog check.
         */
        uint32_t snapshot =
            task_alive_snapshot_and_clear();

        /*
         * Compare the reported activity against the set of tasks currently
         * expected to be alive.
         *
         * Bits outside ALIVE_MASK_EXPECTED are intentionally ignored.
         */
        uint32_t missing =
            ALIVE_MASK_EXPECTED &
            ~snapshot;

        if (missing != 0)
        {
            std::fprintf(
                stderr,
                "TaskWatchdog: CANH BAO - thread khong phan hoi, "
                "missing_mask=0x%02X (",
                missing
            );

            if (
                missing &
                ALIVE_BIT_CAMERA)
            {
                std::fprintf(
                    stderr,
                    "Camera "
                );
            }

            if (
                missing &
                ALIVE_BIT_BALL_DETECT)
            {
                std::fprintf(
                    stderr,
                    "BallDetect "
                );
            }

            if (
                missing &
                ALIVE_BIT_CONTROL_LOOP)
            {
                std::fprintf(
                    stderr,
                    "ControlLoop "
                );
            }

            if (
                missing &
                ALIVE_BIT_CAN_RX)
            {
                std::fprintf(
                    stderr,
                    "CanRx "
                );
            }

            if (
                missing &
                ALIVE_BIT_CAN_TX)
            {
                std::fprintf(
                    stderr,
                    "CanTx "
                );
            }

            std::fprintf(
                stderr,
                ")\n"
            );

            /*
             * The watchdog currently reports and logs missing tasks only.
             *
             * Automatic task recovery is intentionally not performed here
             * because restarting a thread may leave owned resources,
             * communication handles, or shared state in an unsafe condition.
             */
        }
    }
}
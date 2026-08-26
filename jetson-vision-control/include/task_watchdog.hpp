#ifndef TASK_WATCHDOG_HPP
#define TASK_WATCHDOG_HPP

#include <thread>
#include <atomic>
#include <memory>
#include <cstdint>

/*
 * Per-thread heartbeat bits.
 *
 * The bit assignments mirror the STM32 ALIVE_BIT_* definitions so that
 * watchdog status can be interpreted consistently across both platforms.
 */
#define ALIVE_BIT_CAMERA        (1u << 0)
#define ALIVE_BIT_BALL_DETECT   (1u << 1)
#define ALIVE_BIT_CONTROL_LOOP  (1u << 2)
#define ALIVE_BIT_CAN_RX        (1u << 3)
#define ALIVE_BIT_CAN_TX        (1u << 4)

#define ALIVE_MASK_EXPECTED     (ALIVE_BIT_CAMERA | ALIVE_BIT_BALL_DETECT | \
                                 ALIVE_BIT_CONTROL_LOOP | ALIVE_BIT_CAN_RX | \
                                 ALIVE_BIT_CAN_TX)

 /*
  * Global heartbeat API.
  *
  * Tasks call task_alive_mark() directly without holding a TaskWatchdog
  * instance. This keeps watchdog reporting independent from task ownership
  * and thread construction.
  */
void task_alive_mark(uint32_t bit);

uint32_t task_alive_snapshot_and_clear();

/*
 * Periodically checks whether all expected tasks have reported activity.
 *
 * The watchdog only detects and reports missing heartbeats. It does not
 * restart individual threads automatically because recovery requires
 * task-specific handling.
 */
class TaskWatchdog
{
public:
    TaskWatchdog() = default;
    ~TaskWatchdog() { stop(); }

    bool start(uint32_t check_period_ms = 1000);
    void stop();

private:
    void run(uint32_t check_period_ms);

    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> running_{ false };
};

#endif
#ifndef TASK_WATCHDOG_HPP
#define TASK_WATCHDOG_HPP
#include <thread>
#include <atomic>
#include <memory>
#include <cstdint>

/* Bit mask cho tung thread - doi xung ALIVE_BIT_* ben STM32
 * (system_state.h), dung std::atomic<uint32_t> thay osEventFlags. */
#define ALIVE_BIT_CAMERA        (1u << 0)
#define ALIVE_BIT_BALL_DETECT   (1u << 1)
#define ALIVE_BIT_CONTROL_LOOP  (1u << 2)
#define ALIVE_BIT_CAN_RX        (1u << 3)
#define ALIVE_BIT_CAN_TX        (1u << 4)
#define ALIVE_MASK_EXPECTED     (ALIVE_BIT_CAMERA | ALIVE_BIT_BALL_DETECT | \
                                  ALIVE_BIT_CONTROL_LOOP | ALIVE_BIT_CAN_RX | \
                                  ALIVE_BIT_CAN_TX)

/* API global, goi tu bat ky thread nao (giong task_alive_mark() ben
 * STM32) - dat trong system_state de moi thread deu goi duoc ma khong
 * can inject con tro TaskWatchdog vao tung thread. */
void task_alive_mark(uint32_t bit);
uint32_t task_alive_snapshot_and_clear();

/* TaskWatchdog - giam sat dinh ky, KHONG tu dong restart thread (J8 chi
 * lam PHAT HIEN + LOG, chua lam auto-recovery tung thread rieng le - qua
 * phuc tap va rui ro cho pham vi J8). Neu 1 thread "chet" (khong mark
 * trong 1 chu ky), in canh bao ro rang de nguoi van hanh biet. */
class TaskWatchdog {
public:
    TaskWatchdog() = default;
    ~TaskWatchdog() { stop(); }

    bool start(uint32_t check_period_ms = 1000);
    void stop();

private:
    void run(uint32_t check_period_ms);

    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> running_{false};
};

#endif
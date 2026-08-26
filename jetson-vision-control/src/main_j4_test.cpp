/**
 * @file    main_j4_test.cpp
 * @brief   CAN RX/TX integration test application.
 *
 * Starts both CAN reception and transmission tasks on the same CAN interface.
 *
 * The application provides a lightweight runtime monitor for verifying
 * bidirectional communication between Jetson and STM32.
 */

#include "task_can_rx.hpp"
#include "task_can_tx.hpp"
#include "system_state.hpp"

#include <cstdio>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>

static std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a controlled shutdown request so both CAN tasks can
 * be stopped in a defined order.
 */
static void on_sigint(int)
{
    g_stop.store(true);
}

int main()
{
    std::signal(
        SIGINT,
        on_sigint
    );

    TaskCanRx can_rx;
    TaskCanTx can_tx;

    /*
     * Start RX first so incoming STM32 telemetry can be consumed before
     * enabling the periodic TX path.
     */
    if (!can_rx.start("can0"))
    {
        std::printf(
            "Khong start duoc TaskCanRx\n"
        );

        return 1;
    }

    /*
     * If TX startup fails, stop the already-running RX task before exiting
     * so the application does not leave a background CAN worker active.
     */
    if (!can_tx.start("can0"))
    {
        std::printf(
            "Khong start duoc TaskCanTx\n"
        );

        can_rx.stop();

        return 1;
    }

    std::printf(
        "TaskCanRx + TaskCanTx dang chay. Nhan Ctrl+C de dung.\n"
    );

    while (!g_stop.load())
    {
        /*
         * The one-second monitor period is intended only for human-readable
         * diagnostics. CAN RX/TX continue to run at their own task rates.
         */
        std::this_thread::sleep_for(
            std::chrono::seconds(1)
        );

        float roll;
        float pitch;
        float vroll;
        float vpitch;
        float height;

        telemetry_attitude_read(
            system_state().attitude,
            roll,
            pitch,
            vroll,
            vpitch,
            height
        );

        int16_t bx;
        int16_t by;
        int16_t bvx;
        int16_t bvy;

        uint8_t bdet;

        ball_state_read(
            system_state().ball,
            bx,
            by,
            bvx,
            bvy,
            bdet
        );

        /*
         * Display one representative RX value set together with the latest
         * ball state that the TX task publishes to STM32.
         */
        std::printf(
            "[RX] roll=%.2f pitch=%.2f | "
            "[TX] ball_x=%d ball_y=%d detected=%d | "
            "stm32_ok=%d\n",
            roll,
            pitch,
            bx,
            by,
            bdet,
            stm32_state_is_ok()
        );
    }

    /*
     * Stop TX before RX so no new outgoing communication is generated while
     * the CAN receive path is being shut down.
     */
    can_tx.stop();
    can_rx.stop();

    std::printf(
        "Da dung sach.\n"
    );

    return 0;
}
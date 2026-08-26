/**
 * @file    main_j3_test.cpp
 * @brief   CAN RX integration test application.
 *
 * Starts the CAN reception task and periodically displays telemetry received
 * from the STM32 controller.
 *
 * This test validates the CAN RX path and the corresponding SystemState
 * updates without starting the control or vision pipeline.
 */

#include "task_can_rx.hpp"
#include "system_state.hpp"

#include <cstdio>
#include <csignal>
#include <thread>
#include <chrono>
#include <atomic>

static std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a controlled shutdown request so the CAN task can
 * release its resources before the process exits.
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

    /*
     * Start only the CAN RX task so this application isolates the STM32
     * telemetry reception path from the rest of the runtime stack.
     */
    if (!can_rx.start("can0"))
    {
        std::printf(
            "Khong start duoc TaskCanRx\n"
        );

        return 1;
    }

    std::printf(
        "TaskCanRx dang chay. Nhan Ctrl+C de dung.\n"
    );

    while (!g_stop.load())
    {
        /*
         * The one-second reporting period is intentionally much slower than
         * the CAN reception rate because this application is a diagnostic
         * monitor rather than a real-time consumer.
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

        int32_t s1;
        int32_t s2;
        int32_t s3;

        telemetry_servo_read(
            system_state().servo,
            s1,
            s2,
            s3
        );

        uint8_t mode;
        uint8_t bits;

        telemetry_robot_state_read(
            system_state().robot_state,
            mode,
            bits
        );

        /*
         * Display both decoded telemetry and the communication health state
         * so CAN reception can be verified from a single test application.
         */
        std::printf(
            "roll=%.2f pitch=%.2f vroll=%.2f vpitch=%.2f height=%.0f | "
            "S1=%d S2=%d S3=%d | mode=%d bits=0x%02X | stm32_ok=%d\n",
            roll,
            pitch,
            vroll,
            vpitch,
            height,
            s1,
            s2,
            s3,
            mode,
            bits,
            stm32_state_is_ok()
        );
    }

    /*
     * Stop the worker task explicitly before exiting so its CAN socket and
     * thread resources are released deterministically.
     */
    can_rx.stop();

    std::printf(
        "Da dung sach.\n"
    );

    return 0;
}
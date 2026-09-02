/**
 * @file    task_can_rx.cpp
 * @brief   CAN reception task.
 *
 * Receives telemetry frames from STM32, validates the payload length,
 * decodes the protocol fields, and updates the shared system state.
 *
 * Valid STM32 telemetry also refreshes the communication heartbeat used
 * by the runtime safety supervision.
 */

#include "task_can_rx.hpp"
#include "task_watchdog.hpp"
#include "system_state.hpp"
#include "can_protocol.h"

#include <cstdio>
#include <chrono>
#include <pthread.h>

bool TaskCanRx::start(const std::string& ifname)
{
    if (running_.load())
    {
        return true;
    }

    if (!can_.open(ifname))
    {
        std::printf(
            "TaskCanRx: khong mo duoc %s\n",
            ifname.c_str()
        );

        return false;
    }

    running_.store(true);

    thread_ =
        std::make_unique<std::thread>(
            &TaskCanRx::run,
            this
            );

    return true;
}

void TaskCanRx::stop()
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

    can_.close();
}

TaskCanRx::~TaskCanRx()
{
    stop();
}

void TaskCanRx::run()
{
    std::printf(
        "TaskCanRx: STARTED\n"
    );

    /*
     * CAN reception must remain responsive even when CPU-intensive vision
     * processing is active.
     *
     * Keep CAN RX below the real-time control loop priority so the control
     * loop retains the highest scheduling priority while CAN communication
     * still receives preferential CPU access over normal application
     * threads.
     */
    struct sched_param sp;

    sp.sched_priority = 70;

    if (
        pthread_setschedparam(
            pthread_self(),
            SCHED_FIFO,
            &sp) != 0)
    {
        std::fprintf(
            stderr,
            "TaskCanRx: khong set duoc SCHED_FIFO "
            "(can CAP_SYS_NICE/sudo), chay priority mac dinh - "
            "co the bi doi CPU boi TaskControlLoop/TaskBallDetect\n"
        );
    }

    can_frame_t frame;

    uint64_t can_err_count = 0;

    while (running_.load())
    {
        /*
         * Use a bounded receive timeout so the task periodically regains
         * control even when no CAN frame is available.
         *
         * This keeps communication supervision and task liveness checks
         * independent of CAN bus traffic.
         */
        bool got =
            can_.receive(
                frame,
                50
            );

        if (got)
        {
            if (frame.is_error)
            {
                ++can_err_count;

                if (
                    can_err_count <= 20 ||
                    (can_err_count % 100) == 0)
                {
                    std::fprintf(
                        stderr,
                        "[CAN_ERR] class=0x%08X tec=%u rec=%u "
                        "data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                        frame.err_class,
                        (unsigned)frame.data[6],
                        (unsigned)frame.data[7],
                        frame.data[0],
                        frame.data[1],
                        frame.data[2],
                        frame.data[3],
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7]
                    );
                }
            }
            else
            {
            switch (frame.id)
            {
            case CAN_ID_ATTITUDE:
                if (frame.dlc >= 6)
                {
                    float roll =
                        can_rd_i16le(
                            &frame.data[0]
                        ) / 100.0f;

                    float pitch =
                        can_rd_i16le(
                            &frame.data[2]
                        ) / 100.0f;

                    float height =
                        (float)can_rd_i16le(
                            &frame.data[4]
                        );

                    float r;
                    float p;
                    float vr;
                    float vp;
                    float h;

                    telemetry_attitude_read(
                        system_state().attitude,
                        r,
                        p,
                        vr,
                        vp,
                        h
                    );

                    /*
                     * Update only the fields carried by this CAN frame while
                     * preserving the previously received angular rates.
                     */
                    telemetry_attitude_write(
                        system_state().attitude,
                        roll,
                        pitch,
                        vr,
                        vp,
                        height
                    );

                    /*
                     * A valid attitude frame confirms that STM32 is
                     * actively transmitting telemetry.
                     */
                    stm32_heartbeat_mark();
                }

                break;

            case CAN_ID_RATE:
                if (frame.dlc >= 4)
                {
                    float vroll =
                        can_rd_i16le(
                            &frame.data[0]
                        ) / 100.0f;

                    float vpitch =
                        can_rd_i16le(
                            &frame.data[2]
                        ) / 100.0f;

                    float r;
                    float p;
                    float vr;
                    float vp;
                    float h;

                    telemetry_attitude_read(
                        system_state().attitude,
                        r,
                        p,
                        vr,
                        vp,
                        h
                    );

                    /*
                     * Rate telemetry updates only the measured angular
                     * velocities and preserves the latest attitude values.
                     */
                    telemetry_attitude_write(
                        system_state().attitude,
                        r,
                        p,
                        vroll,
                        vpitch,
                        h
                    );
                }

                break;

            case CAN_ID_SERVO_POS:
                if (frame.dlc >= 6)
                {
                    int32_t s1 =
                        can_rd_u16le(
                            &frame.data[0]
                        );

                    int32_t s2 =
                        can_rd_u16le(
                            &frame.data[2]
                        );

                    int32_t s3 =
                        can_rd_u16le(
                            &frame.data[4]
                        );

                    telemetry_servo_write(
                        system_state().servo,
                        s1,
                        s2,
                        s3
                    );

                    /*
                     * Servo telemetry is also a valid indication that the
                     * STM32 communication path is operational.
                     */
                    stm32_heartbeat_mark();
                }

                break;

            case CAN_ID_ROBOT_STATE:
                if (frame.dlc >= 2)
                {
                    telemetry_robot_state_write(
                        system_state().robot_state,
                        frame.data[0],
                        frame.data[1]
                    );

                    /*
                     * Robot-state telemetry is part of the STM32 heartbeat
                     * supervision path.
                     */
                    stm32_heartbeat_mark();
                }

                break;

            case CAN_ID_BALL_DESIRED:
                if (frame.dlc >= 6)
                {
                    int16_t x_d =
                        can_rd_i16le(
                            &frame.data[0]
                        );

                    int16_t y_d =
                        can_rd_i16le(
                            &frame.data[2]
                        );

                    int16_t height_d =
                        can_rd_i16le(
                            &frame.data[4]
                        );

                    /*
                     * POSITION-mode setpoints originate from STM32 and are
                     * published to the shared state for the control loop.
                     */
                    ball_desired_write(
                        system_state().ball_desired,
                        x_d,
                        y_d,
                        height_d
                    );

                    stm32_heartbeat_mark();
                }

                break;

            case CAN_ID_HEARTBEAT_TX:
                /*
                 * The dedicated heartbeat frame does not carry application
                 * telemetry and therefore does not modify the shared state.
                 */
                break;

            default:
                /*
                 * Ignore unsupported CAN identifiers so unrelated bus
                 * traffic cannot affect the control state.
                 */
                break;
            }
            }
        }

        /*
         * Latch communication fault logging so a prolonged STM32
         * disconnection produces one transition message instead of
         * repeatedly flooding the console.
         */
        static bool fault_latched = false;

        bool ok =
            stm32_state_is_ok();

        if (
            !ok &&
            !fault_latched)
        {
            fault_latched = true;

            std::printf(
                "TaskCanRx: FAULT - mat ket noi STM32!\n"
            );
        }
        else if (
            ok &&
            fault_latched)
        {
            fault_latched = false;

            std::printf(
                "TaskCanRx: FAULT CLEARED - STM32 tro lai!\n"
            );
        }

        /*
         * Mark the task alive after completing one receive/supervision
         * iteration. The watchdog therefore supervises both CAN reception
         * and the surrounding task loop.
         */
        task_alive_mark(
            ALIVE_BIT_CAN_RX
        );
    }

    std::printf(
        "TaskCanRx: STOPPED\n"
    );
}
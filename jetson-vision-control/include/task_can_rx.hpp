#ifndef TASK_CAN_RX_HPP
#define TASK_CAN_RX_HPP

#include <thread>
#include <atomic>
#include <memory>

#include "can_transport.hpp"

/*
 * Receives STM32 CAN frames and updates the shared Jetson system state.
 *
 * The task also marks STM32 heartbeat activity when the expected periodic
 * status frames are received. Missing heartbeats can therefore be detected
 * independently of the CAN receive loop.
 *
 * Unlike the STM32 implementation, this task uses a std::thread and a
 * blocking socket read with timeout instead of a FreeRTOS queue/ISR path.
 */
class TaskCanRx
{
public:
    bool start(const std::string& ifname = "can0");

    void stop();

    ~TaskCanRx();

private:
    void run();

    std::atomic<bool> running_{ false };

    CanTransport can_;

    std::unique_ptr<std::thread> thread_;
};

#endif
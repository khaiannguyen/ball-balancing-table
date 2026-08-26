#ifndef TASK_CAN_TX_HPP
#define TASK_CAN_TX_HPP

#include <thread>
#include <atomic>
#include <memory>

#include "can_transport.hpp"

/*
 * Periodically publishes Jetson telemetry and control data over CAN.
 *
 * The task transmits at 100 Hz, including ball position, ball velocity,
 * ball state, desired attitude, and the Jetson heartbeat.
 *
 * Ball data is produced by TaskBallDetect and desired attitude is produced
 * by TaskControlLoop. This task only reads the shared state and transmits
 * it; it does not generate or modify control data.
 */
class TaskCanTx
{
public:
    bool start(const std::string& ifname = "can0");

    void stop();

    ~TaskCanTx();

private:
    void run();

    std::atomic<bool> running_{ false };

    CanTransport can_;

    std::unique_ptr<std::thread> thread_;
};

#endif
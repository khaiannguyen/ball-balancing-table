/**
 * @file    task_control_loop.cpp
 * @brief   Real-time ball balancing control loop.
 *
 * Runs the Jetson-side control loop at a fixed period, selects the active
 * position setpoint according to the STM32 operating mode, computes the
 * desired platform attitude, and publishes the resulting command.
 *
 * The control loop only produces non-zero attitude commands when both the
 * ball measurement and STM32 communication state are valid.
 */

#include "task_control_loop.hpp"

#include "system_state.hpp"

#include "operating_mode.hpp"

#include <cstdio>

#include <chrono>

#include "task_watchdog.hpp"

bool TaskControlLoop::start(
    float kp,
    float ki,
    float kd,
    float out_limit_deg)
{
    if (running_.load(std::memory_order_relaxed))
    {
        std::fprintf(
            stderr,
            "TaskControlLoop: da chay roi\n"
        );

        return false;
    }

    pid_x_ =
        std::make_unique<PIDController>(
            kp,
            ki,
            kd,
            -out_limit_deg,
            out_limit_deg
            );

    pid_y_ =
        std::make_unique<PIDController>(
            kp,
            ki,
            kd,
            -out_limit_deg,
            out_limit_deg
            );

    /*
     * Apply a small error deadband to prevent quantized or low-amplitude
     * position noise from continuously driving the controller when the ball
     * is already close to the requested position.
     */
    pid_x_->set_error_deadband(1.5f);
    pid_y_->set_error_deadband(1.5f);

    /*
     * Filter the measured ball velocity before it is used by the derivative
     * term.
     *
     * The detector already provides velocity information, but an additional
     * EMA filter at the controller boundary limits high-frequency noise
     * reaching the D-term.
     */
    pid_x_->set_velocity_filter_alpha(0.3f);
    pid_y_->set_velocity_filter_alpha(0.3f);

    running_.store(
        true,
        std::memory_order_relaxed
    );

    thread_ =
        std::make_unique<std::thread>(
            &TaskControlLoop::run,
            this
            );

    std::printf(
        "TaskControlLoop: da start "
        "(kp=%.4f ki=%.4f kd=%.4f out_limit=+-%.2fdeg)\n",
        kp,
        ki,
        kd,
        out_limit_deg
    );

    return true;
}

void TaskControlLoop::stop()
{
    if (!running_.load(std::memory_order_relaxed))
    {
        return;
    }

    running_.store(
        false,
        std::memory_order_relaxed
    );

    if (thread_ && thread_->joinable())
    {
        thread_->join();
    }

    std::printf(
        "TaskControlLoop: da dung.\n"
    );
}

TaskControlLoop::~TaskControlLoop()
{
    stop();
}

void TaskControlLoop::run()
{
    using clock =
        std::chrono::steady_clock;

    auto next_wake =
        clock::now();

    /*
     * Run the controller at 100 Hz.
     *
     * The fixed period keeps the controller sample time predictable and
     * matches the expected CAN command update rate.
     */
    const auto period =
        std::chrono::milliseconds(10);

    auto last_time =
        clock::now();

    /*
     * Use a real-time scheduling policy when the process has sufficient
     * privileges.
     *
     * If SCHED_FIFO cannot be enabled, continue with the normal scheduler
     * instead of preventing the application from starting.
     */
    struct sched_param sp;

    sp.sched_priority = 80;

    if (pthread_setschedparam(
        pthread_self(),
        SCHED_FIFO,
        &sp) != 0)
    {
        std::fprintf(
            stderr,
            "TaskControlLoop: khong set duoc SCHED_FIFO "
            "(can CAP_SYS_NICE/sudo), chay priority mac dinh\n"
        );
    }

    while (running_.load(std::memory_order_relaxed))
    {
        next_wake += period;

        auto now =
            clock::now();

        /*
         * Use the actual elapsed time between control iterations rather than
         * assuming the scheduler always wakes at exactly the requested
         * period.
         *
         * This keeps the PID integration and trajectory timing consistent
         * when the thread experiences scheduling jitter.
         */
        float dt =
            std::chrono::duration<float>(
                now - last_time
                ).count();

        last_time = now;

        int16_t x;
        int16_t y;
        int16_t vx;
        int16_t vy;

        uint8_t detected;

        ball_state_read(
            system_state().ball,
            x,
            y,
            vx,
            vy,
            detected
        );

        /*
         * Control output is allowed only when the vision measurement is
         * valid and STM32 communication is still within its heartbeat
         * supervision window.
         */
        bool safe_to_run =
            (detected != 0) &&
            stm32_state_is_ok();

        float roll_d = 0.f;
        float pitch_d = 0.f;

        /*
         * The current operating mode is received from STM32 through the
         * robot-state telemetry and determines which subsystem owns the
         * position setpoint.
         */
        uint8_t mode;
        uint8_t robot_bits;

        telemetry_robot_state_read(
            system_state().robot_state,
            mode,
            robot_bits
        );

        /*
         * Entering BALANCE from another mode starts a new trajectory cycle.
         *
         * Resetting here prevents a previous BALANCE trajectory from being
         * resumed halfway through after the system returns from another mode.
         */
        if (
            mode == opmode::BALANCE &&
            last_mode_ != opmode::BALANCE
            )
        {
            balance_traj_.reset();

            /*
             * Start a new ball-loss supervision window whenever a new
             * BALANCE session begins.
             */
            ball_loss_elapsed_s_ = 0.f;
        }

        last_mode_ = mode;

        /*
         * Select the active position setpoint according to the current mode.
         *
         * BALANCE:
         *   Jetson generates the trajectory setpoint.
         *
         * POSITION:
         *   STM32/user input provides the requested position.
         *
         * Other modes:
         *   Use the safe center position and do not run the trajectory.
         */
        float setpoint_x_mm = 0.f;
        float setpoint_y_mm = 0.f;

        /*
         * Height is currently generated only by the BALANCE trajectory.
         * Other modes retain the default zero-height command.
         */
        float setpoint_height_mm = 0.f;

        if (mode == opmode::BALANCE)
        {
            /*
             * Generate the next trajectory waypoint using the measured
             * control-loop interval.
             */
            Waypoint wp =
                balance_traj_.update(dt);

            setpoint_x_mm =
                wp.x_mm;

            setpoint_y_mm =
                wp.y_mm;

            setpoint_height_mm =
                wp.height_mm;
        }
        else if (mode == opmode::POSITION)
        {
            /*
             * In POSITION mode, STM32 is the source of truth for the
             * requested ball position. Jetson only consumes the value
             * received through the BALL_DESIRED telemetry message.
             */
            int16_t x_d;
            int16_t y_d;
            int16_t height_d_unused;

            ball_desired_read(
                system_state().ball_desired,
                x_d,
                y_d,
                height_d_unused
            );

            setpoint_x_mm =
                (float)x_d;

            setpoint_y_mm =
                (float)y_d;
        }
        else
        {
            /*
             * HOME, CALIBRATION, MANUAL, and other non-positioning modes
             * use the neutral center setpoint.
             *
             * These modes do not consume the BALANCE trajectory or the
             * externally requested ball position.
             */
            setpoint_x_mm = 0.f;
            setpoint_y_mm = 0.f;
        }

        if (safe_to_run)
        {
            /*
             * The detector and mechanical coordinate convention map the
             * measured X position to the pitch command and Y position to
             * the roll command.
             *
             * The velocity measurements are passed directly to the
             * velocity-aware PID update so the D-term can use measured
             * ball motion instead of numerical position differentiation.
             */
            pitch_d =
                pid_x_->update_with_velocity(
                    setpoint_x_mm,
                    (float)x,
                    (float)vx,
                    dt
                );

            roll_d =
                pid_y_->update_with_velocity(
                    setpoint_y_mm,
                    (float)y,
                    (float)vy,
                    dt
                );
        }
        else
        {
            /*
             * Immediately remove controller output when the ball
             * measurement or STM32 heartbeat becomes invalid.
             *
             * Resetting both PID controllers also clears accumulated
             * integral state so stale control effort cannot be restored
             * when valid measurements return.
             */
            pid_x_->reset();
            pid_y_->reset();

            roll_d = 0.f;
            pitch_d = 0.f;

            /*
             * A single missed camera frame should not restart the complete
             * BALANCE trajectory.
             *
             * Accumulate consecutive ball-loss time and reset the trajectory
             * only after the loss exceeds the configured supervision
             * threshold.
             */
            ball_loss_elapsed_s_ += dt;

            if (
                ball_loss_elapsed_s_ >=
                kBallLossResetThresholdS
                )
            {
                balance_traj_.reset();
            }
        }

        /*
         * A valid measurement closes the current ball-loss interval.
         *
         * The next loss event therefore starts a new supervision window
         * instead of accumulating time from an earlier event.
         */
        if (safe_to_run)
        {
            ball_loss_elapsed_s_ = 0.f;
        }

        /*
         * Publish the complete desired attitude as one shared state update.
         *
         * The height command follows the active trajectory while roll and
         * pitch remain controlled by the position PID loops.
         */
        attitude_desired_write(
            system_state().attitude_desired,
            roll_d,
            pitch_d,
            setpoint_height_mm
        );

        std::this_thread::sleep_until(
            next_wake
        );

        /*
         * Report successful completion of one control-loop cycle to the
         * application watchdog.
         */
        task_alive_mark(
            ALIVE_BIT_CONTROL_LOOP
        );
    }
}
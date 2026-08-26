/**
 * @file    task_ball_detect.cpp
 * @brief   Ball detection task.
 *
 * Consumes the latest camera frame, detects the ball position, estimates
 * ball velocity, and publishes the measurement to the shared system state.
 *
 * The task also validates frame freshness and applies measurement
 * conditioning before the control loop consumes the result.
 */

#include "task_ball_detect.hpp"

#include "system_state.hpp"

#include "task_watchdog.hpp"

#include <cstdio>

#include <chrono>
#include <cmath>

bool TaskBallDetect::start(
    TaskCameraCapture& cam,
    BallDetectorConfig cfg,
    const std::string& calib_dir)
{
    if (running_.load(std::memory_order_relaxed))
    {
        std::fprintf(
            stderr,
            "TaskBallDetect: da chay roi\n"
        );

        return false;
    }

    /*
     * The detector requires both camera intrinsics and camera-to-table
     * extrinsic calibration to convert image measurements into physical
     * table coordinates.
     */
    std::string intr_path =
        calib_dir +
        "/intrinsics.yaml";

    std::string extr_path =
        calib_dir +
        "/extrinsic.yaml";

    cv::FileStorage fs_intr(
        intr_path,
        cv::FileStorage::READ
    );

    cv::FileStorage fs_extr(
        extr_path,
        cv::FileStorage::READ
    );

    if (
        !fs_intr.isOpened() ||
        !fs_extr.isOpened()
        )
    {
        std::fprintf(
            stderr,
            "TaskBallDetect: khong doc duoc %s hoac %s — kiem tra "
            "duong dan calib_dir da truyen dung chua.\n",
            intr_path.c_str(),
            extr_path.c_str()
        );

        return false;
    }

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rvec;
    cv::Mat tvec;

    fs_intr["camera_matrix"] >>
        camera_matrix;

    fs_intr["dist_coeffs"] >>
        dist_coeffs;

    fs_extr["rvec"] >>
        rvec;

    fs_extr["tvec"] >>
        tvec;

    fs_intr.release();
    fs_extr.release();

    /*
     * Keep calibration data inside the BallDetector so image processing
     * remains responsible for converting camera measurements into the
     * calibrated physical coordinate system.
     */
    detector_ =
        std::make_unique<BallDetector>(
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            cfg
            );

    cam_ =
        &cam;

    running_.store(
        true,
        std::memory_order_relaxed
    );

    thread_ =
        std::make_unique<std::thread>(
            &TaskBallDetect::run,
            this
            );

    std::printf(
        "TaskBallDetect: da start\n"
    );

    return true;
}

void TaskBallDetect::stop()
{
    if (!running_.load(std::memory_order_relaxed))
    {
        return;
    }

    running_.store(
        false,
        std::memory_order_relaxed
    );

    if (
        thread_ &&
        thread_->joinable()
        )
    {
        thread_->join();
    }

    std::printf(
        "TaskBallDetect: da dung.\n"
    );
}

void TaskBallDetect::run()
{
    cv::Mat frame;

    std::chrono::steady_clock::time_point ts;
    std::chrono::steady_clock::time_point last_ts{};

    bool has_last_ts =
        false;

    double last_x_mm =
        0.0;

    double last_y_mm =
        0.0;

    bool has_last_pos =
        false;

    /*
     * Keep the filtered velocity across iterations so the EMA operates on
     * the continuous measurement history rather than restarting at zero
     * for every frame.
     */
    float vx_filt =
        0.f;

    float vy_filt =
        0.f;

    bool has_filt =
        false;

    /*
     * Require several consecutive missed detections before declaring the
     * ball unavailable.
     *
     * This prevents a single noisy frame, glare event, or temporary
     * detection failure from immediately invalidating the measurement used
     * by the control loop.
     */
    int miss_count =
        0;

    constexpr int MISS_THRESHOLD =
        5;

    while (
        running_.load(
            std::memory_order_relaxed))
    {
        /*
         * Report liveness once per processing-loop iteration.
         *
         * This allows the watchdog to distinguish a running detection task
         * from a task that has become blocked or stopped processing frames.
         */
        task_alive_mark(
            ALIVE_BIT_BALL_DETECT
        );

        if (!cam_->get_latest_frame(
            frame,
            ts))
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5)
            );

            continue;
        }

        /*
         * Process each camera frame at most once.
         *
         * The camera buffer intentionally exposes the latest frame, so the
         * detection loop may observe the same frame multiple times when it
         * runs faster than the camera.
         */
        if (
            has_last_ts &&
            ts == last_ts)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(2)
            );

            continue;
        }

        double x_mm;
        double y_mm;

        bool found =
            detector_->detect(
                frame,
                x_mm,
                y_mm
            );

        if (found)
        {
            /*
             * A valid detection immediately closes the current miss
             * sequence.
             */
            miss_count =
                0;

            if (
                has_last_pos &&
                has_last_ts)
            {
                double dt =
                    std::chrono::duration<double>(
                        ts - last_ts
                        ).count();

                /*
                 * Velocity is updated only when the measurement interval is
                 * physically meaningful.
                 *
                 * Very small dt values can amplify measurement noise,
                 * while an excessively large interval represents a
                 * discontinuity rather than an instantaneous velocity.
                 */
                if (
                    dt > 1e-4 &&
                    dt <= max_valid_dt_s_)
                {
                    float vx_raw =
                        (float)(
                            (x_mm - last_x_mm) /
                            dt
                            );

                    float vy_raw =
                        (float)(
                            (y_mm - last_y_mm) /
                            dt
                            );

                    if (!has_filt)
                    {
                        /*
                         * Initialize the EMA with a scaled first sample
                         * rather than injecting the complete raw velocity
                         * into the controller.
                         *
                         * This limits a possible first-frame velocity spike
                         * after a ball reappears while still allowing the
                         * filter to respond to real motion.
                         */
                        vx_filt =
                            vel_alpha_ *
                            vx_raw;

                        vy_filt =
                            vel_alpha_ *
                            vy_raw;

                        has_filt =
                            true;
                    }
                    else
                    {
                        /*
                         * Apply an exponential moving average to suppress
                         * high-frequency velocity noise before the value
                         * reaches the controller derivative path.
                         *
                         * Lower alpha provides stronger smoothing but
                         * increases response delay.
                         */
                        vx_filt =
                            vel_alpha_ * vx_raw +
                            (1.f - vel_alpha_) * vx_filt;

                        vy_filt =
                            vel_alpha_ * vy_raw +
                            (1.f - vel_alpha_) * vy_filt;
                    }
                }

                /*
                 * If dt is outside the valid range, retain the previous
                 * filtered velocity rather than updating it from a
                 * potentially discontinuous position difference.
                 */
            }

            /*
             * Publish the latest valid position and filtered velocity as
             * separate state fields.
             *
             * Round to the nearest integer before converting to the compact
             * int16 representation used by the shared state.
             */
            ball_state_write_pos(
                system_state().ball,
                (int16_t)x_mm,
                (int16_t)y_mm
            );

            ball_state_write_vel(
                system_state().ball,
                (int16_t)std::lround(vx_filt),
                (int16_t)std::lround(vy_filt)
            );

            ball_state_write_detected(
                system_state().ball,
                1
            );

            last_x_mm =
                x_mm;

            last_y_mm =
                y_mm;

            has_last_pos =
                true;
        }
        else
        {
            /*
             * Do not invalidate the measurement after a single failed
             * detection. The debounce window prevents transient vision
             * noise from producing unnecessary control-state changes.
             */
            miss_count++;

            if (
                miss_count >=
                MISS_THRESHOLD)
            {
                /*
                 * The ball is considered continuously unavailable after the
                 * required number of consecutive misses.
                 *
                 * Clear both position and velocity so downstream tasks
                 * cannot accidentally continue using stale measurements.
                 */
                ball_state_write_pos(
                    system_state().ball,
                    0,
                    0
                );

                ball_state_write_vel(
                    system_state().ball,
                    0,
                    0
                );

                ball_state_write_detected(
                    system_state().ball,
                    0
                );

                /*
                 * Clear the previous position so the next valid detection
                 * starts a new velocity interval instead of producing a
                 * displacement across the ball-loss period.
                 */
                has_last_pos =
                    false;

                /*
                 * Restart velocity filtering after a confirmed ball loss
                 * so velocity from the previous tracking session cannot
                 * influence the first measurements of the next session.
                 */
                has_filt =
                    false;
            }

            /*
             * When miss_count is below the threshold, retain the previous
             * valid state. This prevents a single bad frame from causing
             * position and velocity to flicker between valid and invalid.
             */
        }

        last_ts =
            ts;

        has_last_ts =
            true;
    }
}
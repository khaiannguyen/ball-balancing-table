#pragma once

#include "task_camera_capture.hpp"
#include "ball_detector.hpp"

#include <thread>
#include <atomic>
#include <memory>
#include <string>

/*
 * Runs ball detection on frames provided by TaskCameraCapture.
 *
 * The detected ball state is published to the shared system state for
 * downstream consumers such as TaskControlLoop and TaskCanTx.
 *
 * This task only owns the vision-side ball state update. CAN transmission
 * remains the responsibility of TaskCanTx.
 */
class TaskBallDetect
{
public:
    TaskBallDetect() = default;
    ~TaskBallDetect() { stop(); }

    TaskBallDetect(const TaskBallDetect&) = delete;
    TaskBallDetect& operator=(const TaskBallDetect&) = delete;

    /*
     * The camera must be started successfully before this task is started.
     *
     * Calibration files are loaded from calib_dir. The default path assumes
     * the application is launched from the project working directory.
     */
    bool start(
        TaskCameraCapture& cam,
        BallDetectorConfig cfg = {},
        const std::string& calib_dir = "calib"
    );

    void stop();

    bool is_running() const
    {
        return running_.load(
            std::memory_order_relaxed
        );
    }

    /*
     * Sets the EMA coefficient used for ball velocity filtering.
     *
     * Lower alpha provides stronger smoothing but increases response delay.
     * alpha = 1.0 disables filtering.
     */
    void set_velocity_filter_alpha(
        float alpha
    )
    {
        vel_alpha_ = alpha;
    }

    /*
     * Defines the maximum valid interval between consecutive ball detections
     * for velocity estimation.
     *
     * If the interval exceeds this value, the velocity update is skipped
     * instead of generating an invalid spike from a long detection gap.
     */
    void set_max_valid_dt(
        double max_dt_s
    )
    {
        max_valid_dt_s_ = max_dt_s;
    }

private:
    void run();

    TaskCameraCapture* cam_ = nullptr;

    std::unique_ptr<BallDetector> detector_;

    std::unique_ptr<std::thread> thread_;

    std::atomic<bool> running_{ false };

    float vel_alpha_ = 0.35f;

    double max_valid_dt_s_ = 0.15;
};
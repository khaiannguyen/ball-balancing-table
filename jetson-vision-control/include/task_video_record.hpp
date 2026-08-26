#pragma once

#include "task_camera_capture.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <fstream>
#include <string>
#include <chrono>

/*
 * Records raw camera frames and a timestamp for each frame.
 *
 * Video recording is intentionally independent of ball detection, control,
 * and CAN processing. Frames are consumed from the same FrameBox used by
 * other readers, so recording does not participate in the control pipeline.
 *
 * Two files are produced:
 *
 * 1. video_path:
 *    Raw video frames without overlays.
 *
 * 2. timestamps_csv_path:
 *    frame_index and timestamp_ms for each recorded frame.
 *
 * timestamp_ms uses the same t_start reference as data.csv so recorded
 * frames can be synchronized with control and setpoint data offline.
 * The recorded timestamps are authoritative; fps_hint is only used as
 * the nominal video playback rate.
 */
class TaskVideoRecord
{
public:
    TaskVideoRecord() = default;
    ~TaskVideoRecord() { stop(); }

    TaskVideoRecord(const TaskVideoRecord&) = delete;
    TaskVideoRecord& operator=(const TaskVideoRecord&) = delete;

    /*
     * The camera must be started successfully before recording begins.
     *
     * t_start must use the same time reference used for data.csv.
     * fps_hint defines the nominal video playback rate and does not
     * determine timestamp synchronization.
     */
    bool start(
        TaskCameraCapture& cam,
        const std::string& video_path,
        const std::string& timestamps_csv_path,
        std::chrono::steady_clock::time_point t_start,
        double fps_hint = 30.0
    );

    void stop();

    bool is_running() const
    {
        return running_.load(
            std::memory_order_relaxed
        );
    }

private:
    void run();

    TaskCameraCapture* cam_ = nullptr;

    std::unique_ptr<std::thread> thread_;

    std::atomic<bool> running_{ false };

    cv::VideoWriter writer_;
    std::ofstream ts_csv_;

    std::string video_path_;

    double fps_hint_ = 30.0;

    std::chrono::steady_clock::time_point t_start_{};
};
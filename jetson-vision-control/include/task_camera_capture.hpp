#pragma once

#include "camera_pipeline.hpp"

#include <opencv2/opencv.hpp>

#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

/*
 * Double-buffered frame storage shared by camera producers and consumers.
 *
 * ready_idx identifies the buffer containing the latest completed frame:
 *   0 or 1 = valid buffer
 *   -1     = no frame available yet
 *
 * The writer always publishes into the buffer opposite to ready_idx.
 * This prevents the writer from overwriting the buffer currently exposed
 * to readers during normal operation.
 *
 * Each buffer has its own mutex. The mutex protects cv::Mat assignment and
 * timestamp access because cv::Mat metadata is not safe to read and write
 * concurrently.
 *
 * A seqlock is intentionally not used here because cv::Mat contains multiple
 * fields that cannot be updated atomically as one object.
 */
struct FrameBox
{
    cv::Mat buffers[2];

    std::chrono::steady_clock::time_point timestamps[2];

    std::mutex mtx[2];

    std::atomic<int> ready_idx{ -1 };

    // Called by the camera callback thread.
    void publish(
        cv::Mat&& frame,
        std::chrono::steady_clock::time_point ts
    )
    {
        int cur =
            ready_idx.load(
                std::memory_order_acquire
            );

        int write_idx =
            (cur == 0) ? 1 : 0;

        {
            std::lock_guard<std::mutex> lk(
                mtx[write_idx]
            );

            buffers[write_idx] =
                std::move(frame);

            timestamps[write_idx] =
                ts;
        }

        ready_idx.store(
            write_idx,
            std::memory_order_release
        );
    }

    /*
     * Returns the latest published frame without waiting for a new frame.
     *
     * cv::Mat assignment is inexpensive because it shares the underlying
     * pixel buffer through reference counting. The selected buffer remains
     * protected while the output header and timestamp are copied.
     */
    bool get_latest(
        cv::Mat& out,
        std::chrono::steady_clock::time_point& ts
    )
    {
        int idx =
            ready_idx.load(
                std::memory_order_acquire
            );

        if (idx < 0)
        {
            return false;
        }

        std::lock_guard<std::mutex> lk(
            mtx[idx]
        );

        out =
            buffers[idx];

        ts =
            timestamps[idx];

        return true;
    }
};

/*
 * Owns the camera pipeline used by the main runtime.
 *
 * Captured frames are published into FrameBox for independent consumers
 * such as TaskBallDetect and TaskVideoRecord.
 */
class TaskCameraCapture
{
public:
    TaskCameraCapture() = default;
    ~TaskCameraCapture() { stop(); }

    TaskCameraCapture(const TaskCameraCapture&) = delete;
    TaskCameraCapture& operator=(const TaskCameraCapture&) = delete;

    /*
     * Opens the camera pipeline and starts frame capture.
     *
     * Returns false if CameraPipeline::open() fails.
     */
    bool start(
        int width,
        int height,
        int fps,
        int sensor_id = 0
    );

    /*
     * Stops capture and closes the pipeline.
     *
     * Frame capture is driven by the pipeline's streaming callback, so this
     * class does not own a separate worker thread that requires joining.
     */
    void stop();

    bool is_running() const
    {
        return running_.load(
            std::memory_order_relaxed
        );
    }

    /*
     * Returns the latest captured frame and its capture timestamp.
     * Returns false until at least one frame has been published.
     */
    bool get_latest_frame(
        cv::Mat& out,
        std::chrono::steady_clock::time_point& ts
    )
    {
        return box_.get_latest(
            out,
            ts
        );
    }

    // Exposes camera timing statistics for runtime diagnostics.
    const JitterStats& jitter_stats() const
    {
        return pipeline_.jitter_stats();
    }

private:
    CameraPipeline pipeline_;

    FrameBox box_;

    std::atomic<bool> running_{ false };
};
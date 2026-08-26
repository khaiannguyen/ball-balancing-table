#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <string>

/*
 * Frame timing statistics collected from the camera pipeline.
 *
 * Welford's online algorithm is used to calculate the running mean and
 * variance without accumulating all samples or suffering from large
 * intermediate sums.
 */
struct JitterStats
{
    long n = 0;

    double mean = 0.0;
    double m2 = 0.0;

    double last_dt_ms = 0.0;

    /*
     * Adds one frame interval sample in milliseconds.
     */
    void add_sample(
        double dt_ms)
    {
        last_dt_ms =
            dt_ms;

        n++;

        double delta =
            dt_ms - mean;

        mean +=
            delta /
            static_cast<double>(n);

        m2 +=
            delta *
            (dt_ms - mean);
    }

    /*
     * Returns the population standard deviation of the collected frame
     * intervals.
     *
     * Returns zero until at least two samples are available.
     */
    double stddev() const
    {
        return (n > 1)
            ? std::sqrt(
                m2 /
                static_cast<double>(n)
            )
            : 0.0;
    }
};

/*
 * GStreamer-based camera capture pipeline.
 *
 * Frames are delivered through the callback as OpenCV BGR images together
 * with the monotonic timestamp recorded when the sample becomes available.
 *
 * The timestamp allows downstream tasks to measure frame-to-frame jitter and
 * processing latency without depending on wall-clock time.
 */
class CameraPipeline
{
public:
    using FrameCallback =
        std::function<
        void(
            cv::Mat&& frame,
            std::chrono::steady_clock::time_point ts
            )
        >;

    CameraPipeline() = default;

    ~CameraPipeline()
    {
        close();
    }

    /*
     * Non-copyable because the class owns GStreamer pipeline resources.
     */
    CameraPipeline(
        const CameraPipeline&) = delete;

    CameraPipeline& operator=(
        const CameraPipeline&) = delete;

    /*
     * Opens the selected camera and creates the GStreamer pipeline.
     *
     * width and height define the requested frame resolution.
     * fps defines the requested capture rate.
     * sensor_id selects the camera source when multiple sensors are present.
     */
    bool open(
        int width,
        int height,
        int fps,
        int sensor_id = 0);

    /*
     * Registers the callback invoked when a new frame is available.
     *
     * The callback receives ownership of the OpenCV frame through an rvalue
     * reference to avoid an unnecessary image copy.
     */
    void set_callback(
        FrameCallback cb)
    {
        callback_ =
            std::move(cb);
    }

    /*
     * Stops the pipeline and releases all GStreamer resources.
     */
    void close();

    /*
     * Returns true when the GStreamer pipeline is currently initialized.
     */
    bool is_open() const
    {
        return pipeline_ != nullptr;
    }

    /*
     * Returns the current frame timing statistics.
     */
    const JitterStats& jitter_stats() const
    {
        return jitter_;
    }

private:
    /*
     * GStreamer appsink callback used to receive completed camera samples.
     */
    static GstFlowReturn on_new_sample(
        GstAppSink* sink,
        gpointer user_data);

    /*
     * Converts a GStreamer sample into an OpenCV frame and dispatches it
     * through the registered callback.
     */
    void handle_sample(
        GstSample* sample);

    GstElement* pipeline_ = nullptr;

    GstElement* appsink_ = nullptr;

    FrameCallback callback_;

    JitterStats jitter_;

    /*
     * Timestamp of the previous completed frame.
     */
    std::chrono::steady_clock::time_point last_ts_{};

    bool has_last_ts_ = false;

    int width_ = 0;

    int height_ = 0;
};
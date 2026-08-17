#pragma once
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <functional>
#include <chrono>
#include <cmath>
#include <string>

// Thống kê jitter, dùng thuật toán Welford (tránh cộng dồn tràn số qua thời gian dài)
struct JitterStats {
    long   n      = 0;
    double mean   = 0.0;
    double m2     = 0.0;
    double last_dt_ms = 0.0;

    void add_sample(double dt_ms) {
        last_dt_ms = dt_ms;
        n++;
        double delta = dt_ms - mean;
        mean += delta / static_cast<double>(n);
        m2   += delta * (dt_ms - mean);
    }
    double stddev() const {
        return (n > 1) ? std::sqrt(m2 / static_cast<double>(n)) : 0.0;
    }
};

class CameraPipeline {
public:
    // callback nhận: frame BGR, timestamp lúc buffer sẵn sàng (dùng để đo jitter/latency phía sau)
    using FrameCallback =
        std::function<void(cv::Mat&& frame, std::chrono::steady_clock::time_point ts)>;

    CameraPipeline() = default;
    ~CameraPipeline() { close(); }

    // Không cho copy (giữ handle GStreamer duy nhất)
    CameraPipeline(const CameraPipeline&)            = delete;
    CameraPipeline& operator=(const CameraPipeline&) = delete;

    bool open(int width, int height, int fps, int sensor_id = 0);
    void set_callback(FrameCallback cb) { callback_ = std::move(cb); }
    void close();

    bool is_open() const { return pipeline_ != nullptr; }
    const JitterStats& jitter_stats() const { return jitter_; }

private:
    static GstFlowReturn on_new_sample(GstAppSink* sink, gpointer user_data);
    void handle_sample(GstSample* sample);

    GstElement* pipeline_ = nullptr;
    GstElement* appsink_  = nullptr;

    FrameCallback callback_;
    JitterStats   jitter_;
    std::chrono::steady_clock::time_point last_ts_{};
    bool has_last_ts_ = false;

    int width_  = 0;
    int height_ = 0;
};
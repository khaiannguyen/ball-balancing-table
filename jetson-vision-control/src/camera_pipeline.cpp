#include "camera_pipeline.hpp"
#include <sstream>
#include <cstdio>
#include <cstring>

bool CameraPipeline::open(int width, int height, int fps, int sensor_id) {
    width_  = width;
    height_ = height;

    // gst_init an toàn khi gọi nhiều lần (idempotent theo tài liệu GStreamer)
    if (!gst_is_initialized()) {
        gst_init(nullptr, nullptr);
    }

    std::ostringstream ss;
    ss << "nvarguscamerasrc sensor-id=" << sensor_id << " ! "
       << "video/x-raw(memory:NVMM),width=" << width
       << ",height=" << height
       << ",framerate=" << fps << "/1 ! "
       << "nvvidconv ! video/x-raw,format=BGRx ! "
       << "videoconvert ! video/x-raw,format=BGR ! "
       << "appsink name=camsink emit-signals=false sync=false "
       << "max-buffers=1 drop=true";

    std::string pipeline_str = ss.str();
    std::printf("GStreamer pipeline: %s\n", pipeline_str.c_str());

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    if (err != nullptr) {
        std::fprintf(stderr, "Loi parse pipeline: %s\n", err->message);
        g_error_free(err);
        pipeline_ = nullptr;
        return false;
    }

    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "camsink");
    if (appsink_ == nullptr) {
        std::fprintf(stderr, "Khong tim thay element 'camsink'\n");
        close();
        return false;
    }

    // Set callback qua GstAppSinkCallbacks — new_sample được gọi ngay trong
    // GStreamer streaming thread khi frame sẵn sàng. Đây chính là nơi lấy
    // timestamp chính xác nhất để đo jitter (không qua hàng đợi consumer).
    GstAppSinkCallbacks callbacks = {};
    callbacks.new_sample = &CameraPipeline::on_new_sample;
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        std::fprintf(stderr, "Khong the chuyen pipeline sang PLAYING\n");
        close();
        return false;
    }

    return true;
}

GstFlowReturn CameraPipeline::on_new_sample(GstAppSink* sink, gpointer user_data) {
    auto* self = static_cast<CameraPipeline*>(user_data);
    GstSample* sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
        return GST_FLOW_ERROR;
    }
    self->handle_sample(sample);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void CameraPipeline::handle_sample(GstSample* sample) {
    // ---- Đo jitter: timestamp ngay khi buffer tới tay callback ----
    auto now = std::chrono::steady_clock::now();
    if (has_last_ts_) {
        double dt_ms = std::chrono::duration<double, std::milli>(now - last_ts_).count();
        jitter_.add_sample(dt_ms);
    }
    last_ts_ = now;
    has_last_ts_ = true;

    if (!callback_) return; // không ai đăng ký nhận frame, chỉ đo jitter thôi cũng được

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (buffer == nullptr) return;

    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* s = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        std::fprintf(stderr, "gst_buffer_map that bai\n");
        return;
    }

    // BGR 8UC3, mỗi hàng step = width*3 (không có padding trong pipeline này)
    cv::Mat frame(h, w, CV_8UC3, map.data);
    cv::Mat frame_copy = frame.clone(); // clone bắt buộc: map.data chỉ hợp lệ trong scope này

    gst_buffer_unmap(buffer, &map);

    callback_(std::move(frame_copy), now);
}

void CameraPipeline::close() {
    if (pipeline_ != nullptr) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (appsink_ != nullptr) {
            gst_object_unref(appsink_);
            appsink_ = nullptr;
        }
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}
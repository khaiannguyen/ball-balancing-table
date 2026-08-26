/**
 * @file    camera_pipeline.cpp
 * @brief   GStreamer camera pipeline implementation.
 *
 * Provides the low-level camera pipeline used by the Jetson vision stack.
 *
 * The pipeline acquires frames through nvarguscamerasrc, converts them to
 * OpenCV-compatible BGR images, and forwards each frame to the registered
 * callback together with a monotonic timestamp.
 *
 * The pipeline intentionally keeps only the latest frame so downstream
 * processing does not accumulate stale camera data.
 */

#include "camera_pipeline.hpp"

#include <sstream>

#include <cstdio>
#include <cstring>

bool CameraPipeline::open(
    int width,
    int height,
    int fps,
    int sensor_id)
{
    width_ = width;
    height_ = height;

    /*
     * GStreamer initialization is process-wide and safe to perform only
     * when it has not already been initialized by another component.
     */
    if (!gst_is_initialized())
    {
        gst_init(
            nullptr,
            nullptr
        );
    }

    std::ostringstream ss;

    /*
     * Use the NVIDIA Argus camera source and keep the camera buffer in
     * NVMM memory until the hardware-accelerated conversion stage.
     *
     * The final BGR conversion provides the format expected by OpenCV
     * and the ball-detection pipeline.
     */
    ss
        << "nvarguscamerasrc sensor-id="
        << sensor_id
        << " ! "
        << "video/x-raw(memory:NVMM),width="
        << width
        << ",height="
        << height
        << ",framerate="
        << fps
        << "/1 ! "
        << "nvvidconv ! video/x-raw,format=BGRx ! "
        << "videoconvert ! video/x-raw,format=BGR ! "
        << "appsink name=camsink emit-signals=false sync=false "
        << "max-buffers=1 drop=true";

    std::string pipeline_str =
        ss.str();

    std::printf(
        "GStreamer pipeline: %s\n",
        pipeline_str.c_str()
    );

    GError* err = nullptr;

    pipeline_ =
        gst_parse_launch(
            pipeline_str.c_str(),
            &err
        );

    if (err != nullptr)
    {
        std::fprintf(
            stderr,
            "Loi parse pipeline: %s\n",
            err->message
        );

        g_error_free(err);

        pipeline_ =
            nullptr;

        return false;
    }

    appsink_ =
        gst_bin_get_by_name(
            GST_BIN(pipeline_),
            "camsink"
        );

    if (appsink_ == nullptr)
    {
        std::fprintf(
            stderr,
            "Khong tim thay element 'camsink'\n"
        );

        close();

        return false;
    }

    /*
     * Register the sample callback before starting the pipeline.
     *
     * GStreamer invokes the callback from its streaming thread when a new
     * sample becomes available. This provides the earliest practical point
     * in the application pipeline for measuring frame-arrival timing.
     */
    GstAppSinkCallbacks callbacks = {};

    callbacks.new_sample =
        &CameraPipeline::on_new_sample;

    gst_app_sink_set_callbacks(
        GST_APP_SINK(appsink_),
        &callbacks,
        this,
        nullptr
    );

    GstStateChangeReturn ret =
        gst_element_set_state(
            pipeline_,
            GST_STATE_PLAYING
        );

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        std::fprintf(
            stderr,
            "Khong the chuyen pipeline sang PLAYING\n"
        );

        close();

        return false;
    }

    return true;
}

GstFlowReturn CameraPipeline::on_new_sample(
    GstAppSink* sink,
    gpointer user_data)
{
    auto* self =
        static_cast<CameraPipeline*>(user_data);

    GstSample* sample =
        gst_app_sink_pull_sample(sink);

    if (sample == nullptr)
    {
        return GST_FLOW_ERROR;
    }

    self->handle_sample(sample);

    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

void CameraPipeline::handle_sample(
    GstSample* sample)
{
    /*
     * Timestamp the frame as soon as it reaches the callback.
     *
     * Measuring at this boundary captures the actual frame-arrival interval
     * seen by the application without including downstream queueing or
     * processing latency.
     */
    auto now =
        std::chrono::steady_clock::now();

    if (has_last_ts_)
    {
        double dt_ms =
            std::chrono::duration<double, std::milli>(
                now - last_ts_
                ).count();

        jitter_.add_sample(
            dt_ms
        );
    }

    last_ts_ =
        now;

    has_last_ts_ =
        true;

    /*
     * Jitter measurement remains useful even when no frame consumer is
     * registered. In that case the sample can be discarded after timing
     * information has been updated.
     */
    if (!callback_)
    {
        return;
    }

    GstBuffer* buffer =
        gst_sample_get_buffer(sample);

    if (buffer == nullptr)
    {
        return;
    }

    GstCaps* caps =
        gst_sample_get_caps(sample);

    GstStructure* s =
        gst_caps_get_structure(
            caps,
            0
        );

    int w = 0;
    int h = 0;

    gst_structure_get_int(
        s,
        "width",
        &w
    );

    gst_structure_get_int(
        s,
        "height",
        &h
    );

    GstMapInfo map;

    if (!gst_buffer_map(
        buffer,
        &map,
        GST_MAP_READ))
    {
        std::fprintf(
            stderr,
            "gst_buffer_map that bai\n"
        );

        return;
    }

    /*
     * The pipeline produces tightly packed BGR 8-bit pixels, so the mapped
     * buffer can be exposed directly as an OpenCV CV_8UC3 image while the
     * GStreamer buffer remains mapped.
     */
    cv::Mat frame(
        h,
        w,
        CV_8UC3,
        map.data
    );

    /*
     * The mapped GStreamer memory is valid only while the buffer remains
     * mapped. Clone the image before unmapping so the callback receives
     * independent storage with a valid lifetime.
     */
    cv::Mat frame_copy =
        frame.clone();

    gst_buffer_unmap(
        buffer,
        &map
    );

    callback_(
        std::move(frame_copy),
        now
    );
}

void CameraPipeline::close()
{
    if (pipeline_ != nullptr)
    {
        /*
         * Stop the pipeline before releasing its GStreamer objects so the
         * streaming thread no longer produces samples during teardown.
         */
        gst_element_set_state(
            pipeline_,
            GST_STATE_NULL
        );

        if (appsink_ != nullptr)
        {
            gst_object_unref(
                appsink_
            );

            appsink_ =
                nullptr;
        }

        gst_object_unref(
            pipeline_
        );

        pipeline_ =
            nullptr;
    }
}
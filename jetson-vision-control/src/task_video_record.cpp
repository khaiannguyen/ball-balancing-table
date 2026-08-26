/**
 * @file    task_video_record.cpp
 * @brief   Camera frame recording and timestamp logging task.
 *
 * Records the latest camera frames to a video file and stores the
 * corresponding capture timestamps in a CSV file.
 *
 * The timestamp stream is kept independently from video encoding so the
 * recorded frame timing can still be recovered when video encoding fails.
 */

#include "task_video_record.hpp"

#include <cstdio>

bool TaskVideoRecord::start(
    TaskCameraCapture& cam,
    const std::string& video_path,
    const std::string& timestamps_csv_path,
    std::chrono::steady_clock::time_point t_start,
    double fps_hint)
{
    if (
        running_.load(
            std::memory_order_relaxed))
    {
        std::fprintf(
            stderr,
            "TaskVideoRecord: da chay roi\n"
        );

        return false;
    }

    cam_ =
        &cam;

    video_path_ =
        video_path;

    fps_hint_ =
        fps_hint;

    t_start_ =
        t_start;

    /*
     * Timestamp logging is required for offline synchronization between the
     * recorded video and other time-stamped system data.
     *
     * Fail startup if the timestamp file cannot be created because a video
     * without synchronization information is not sufficient for later
     * analysis.
     */
    ts_csv_.open(
        timestamps_csv_path,
        std::ios::out |
        std::ios::trunc
    );

    if (!ts_csv_.is_open())
    {
        std::fprintf(
            stderr,
            "TaskVideoRecord: khong mo duoc %s de ghi "
            "timestamps, HUY start (khong ghi video ma khong co "
            "timestamp thi video vo nghia de dong bo lai sau).\n",
            timestamps_csv_path.c_str()
        );

        return false;
    }

    ts_csv_
        << "frame_index,timestamp_ms\n";

    ts_csv_.flush();

    /*
     * Open VideoWriter lazily when the first frame arrives so the actual
     * frame dimensions produced by the camera pipeline are used.
     *
     * This avoids assuming that the requested camera dimensions are
     * necessarily identical to the dimensions delivered by GStreamer.
     */
    running_.store(
        true,
        std::memory_order_relaxed
    );

    thread_ =
        std::make_unique<std::thread>(
            &TaskVideoRecord::run,
            this
            );

    std::printf(
        "TaskVideoRecord: da start - video: %s, timestamps: %s\n",
        video_path.c_str(),
        timestamps_csv_path.c_str()
    );

    return true;
}

void TaskVideoRecord::stop()
{
    if (
        !running_.load(
            std::memory_order_relaxed))
    {
        return;
    }

    running_.store(
        false,
        std::memory_order_relaxed
    );

    if (
        thread_ &&
        thread_->joinable())
    {
        thread_->join();
    }

    /*
     * Release the video encoder before closing the timestamp stream so all
     * pending video resources are finalized during task shutdown.
     */
    if (writer_.isOpened())
    {
        writer_.release();
    }

    if (ts_csv_.is_open())
    {
        ts_csv_.close();
    }

    std::printf(
        "TaskVideoRecord: da dung.\n"
    );
}

void TaskVideoRecord::run()
{
    cv::Mat frame;

    std::chrono::steady_clock::time_point ts;
    std::chrono::steady_clock::time_point last_ts{};

    bool has_last_ts =
        false;

    int64_t frame_index =
        0;

    /*
     * Prevent repeated VideoWriter initialization attempts after a codec
     * failure. Timestamp logging continues even when video encoding is
     * unavailable.
     */
    bool writer_failed_once =
        false;

    while (
        running_.load(
            std::memory_order_relaxed))
    {
        /*
         * Consume the latest available camera frame.
         *
         * A short wait avoids busy-spinning while the camera has not
         * produced a new frame.
         */
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
         * The camera buffer exposes the latest frame rather than a queued
         * sequence. Ignore an unchanged timestamp so one camera frame is
         * never recorded multiple times when this task runs faster than
         * the camera.
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

        /*
         * Open the writer only after receiving the first real frame so its
         * dimensions exactly match the camera output.
         */
        if (
            !writer_.isOpened() &&
            !writer_failed_once)
        {
            int fourcc =
                cv::VideoWriter::fourcc(
                    'm',
                    'p',
                    '4',
                    'v'
                );

            writer_.open(
                video_path_,
                fourcc,
                fps_hint_,
                frame.size(),
                /*isColor=*/true
            );

            if (!writer_.isOpened())
            {
                std::fprintf(
                    stderr,
                    "TaskVideoRecord: KHONG mo duoc "
                    "VideoWriter cho %s (kiem tra codec mp4v co "
                    "san khong, hoac doi duoi .avi). Van tiep tuc "
                    "ghi timestamps.csv nhung SE KHONG co video.\n",
                    video_path_.c_str()
                );

                /*
                 * Do not retry every frame after a codec initialization
                 * failure. Repeated initialization attempts would waste
                 * CPU while producing the same failure condition.
                 */
                writer_failed_once =
                    true;
            }
        }

        if (writer_.isOpened())
        {
            /*
             * CameraPipeline provides BGR frames, which is the color layout
             * expected by the OpenCV VideoWriter configuration used here.
             */
            writer_.write(
                frame
            );
        }

        /*
         * Store the camera timestamp relative to the recording start time.
         *
         * This provides a common time reference for offline synchronization
         * with other system data recorded from the same monotonic clock.
         */
        double t_ms =
            std::chrono::duration<double, std::milli>(
                ts - t_start_
                ).count();

        ts_csv_
            << frame_index
            << ","
            << t_ms
            << "\n";

        /*
         * Flush each frame so timestamp information is persisted promptly
         * even if the application terminates unexpectedly.
         */
        ts_csv_.flush();

        frame_index++;

        last_ts =
            ts;

        has_last_ts =
            true;
    }
}
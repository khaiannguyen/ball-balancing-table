/**
 * @file    main_j6_camera_test.cpp
 * @brief   Camera capture and frame-timing test application.
 *
 * Starts the camera capture task and periodically retrieves the latest frame.
 *
 * The test measures frame availability and reports camera jitter statistics
 * without running the ball-detection pipeline.
 */

#include "task_camera_capture.hpp"

#include <csignal>
#include <atomic>
#include <cstdio>
#include <thread>
#include <chrono>

static std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a controlled shutdown request so the camera pipeline
 * can be closed cleanly.
 */
static void on_sigint(int)
{
    g_stop.store(true);
}

int main()
{
    std::signal(
        SIGINT,
        on_sigint
    );

    TaskCameraCapture cam;

    /*
     * Use the production camera configuration while intentionally consuming
     * frames at a much lower diagnostic rate.
     */
    if (!cam.start(
        1280,
        720,
        60,
        /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Khong the start camera capture\n"
        );

        return 1;
    }

    cv::Mat frame;

    std::chrono::steady_clock::time_point ts;

    int got_count =
        0;

    int miss_count =
        0;

    while (!g_stop.load())
    {
        /*
         * Retrieve only the latest frame. This test intentionally does not
         * attempt to consume the complete 60 FPS stream.
         */
        if (cam.get_latest_frame(
            frame,
            ts))
        {
            got_count++;

            /*
             * Report periodically rather than for every frame so console
             * output does not become the dominant workload of the test.
             */
            if (
                got_count % 30 ==
                0)
            {
                const auto& js =
                    cam.jitter_stats();

                std::printf(
                    "[cam] got=%d miss=%d frame=%dx%d "
                    "jitter mean=%.2fms stddev=%.2fms\n",
                    got_count,
                    miss_count,
                    frame.cols,
                    frame.rows,
                    js.mean,
                    js.stddev()
                );
            }
        }
        else
        {
            /*
             * A miss is normally expected only before the first camera frame
             * becomes available or during a temporary capture condition.
             */
            miss_count++;
        }

        /*
         * Poll at approximately 10 Hz because this application is a camera
         * diagnostic tool rather than the real-time frame consumer.
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    std::printf(
        "Dung sach... got=%d miss=%d\n",
        got_count,
        miss_count
    );

    /*
     * Stop the camera pipeline after the consumer loop has terminated.
     */
    cam.stop();

    return 0;
}
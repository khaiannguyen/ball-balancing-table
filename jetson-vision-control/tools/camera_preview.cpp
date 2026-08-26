/**
 * @file    camera_preview.cpp
 * @brief   Live camera preview and framing test.
 *
 * Displays the camera stream when a graphical display is available.
 *
 * When running headless, periodically saves the latest frame to a temporary
 * JPEG file so the camera view can still be inspected remotely.
 */

#include "camera_pipeline.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a controlled shutdown request.
 */
void on_sigint(int)
{
    g_stop.store(true);
}

int main()
{
    std::signal(
        SIGINT,
        on_sigint
    );

    CameraPipeline cam;

    if (!cam.open(
        1280,
        720,
        60,
        /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Failed to open camera.\n"
        );

        return 1;
    }

    const bool has_display =
        (std::getenv("DISPLAY") != nullptr);

    if (!has_display)
    {
        std::printf(
            "No DISPLAY detected (SSH/headless mode). "
            "Saving /tmp/preview_latest.jpg approximately every 0.5s.\n"
        );
    }

    std::atomic<int> frame_idx{ 0 };

    cam.set_callback(
        [&](cv::Mat&& frame, auto /*ts*/)
        {
            /*
             * Draw a center marker to simplify physical camera alignment
             * with the balancing table.
             */
            cv::Point center(
                frame.cols / 2,
                frame.rows / 2
            );

            cv::drawMarker(
                frame,
                center,
                cv::Scalar(0, 0, 255),
                cv::MARKER_CROSS,
                40,
                2
            );

            cv::circle(
                frame,
                center,
                5,
                cv::Scalar(0, 255, 0),
                -1
            );

            int idx =
                frame_idx.fetch_add(1);

            if (has_display)
            {
                cv::imshow(
                    "preview",
                    frame
                );

                cv::waitKey(1);
            }
            else if (idx % 30 == 0)
            {
                /*
                 * At 60 FPS, saving every 30 frames provides approximately
                 * two preview images per second in headless mode.
                 */
                cv::imwrite(
                    "/tmp/preview_latest.jpg",
                    frame
                );
            }
        }
    );

    std::printf(
        "Camera preview is running. Press Ctrl+C to stop.\n"
    );

    while (!g_stop.load())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    cam.close();

    std::printf(
        "Camera preview stopped cleanly.\n"
    );

    return 0;
}
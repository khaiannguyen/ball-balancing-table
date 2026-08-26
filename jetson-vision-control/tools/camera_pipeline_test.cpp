/**
 * @file    camera_pipeline_test.cpp
 * @brief   Camera pipeline and frame-timing test.
 *
 * Opens the camera at the target capture configuration and reports frame
 * count and jitter statistics during runtime.
 *
 * This tool isolates the low-level camera pipeline from the rest of the
 * vision processing stack.
 */

#include "camera_pipeline.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
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

    /*
     * Use the production camera configuration so the test reflects the
     * expected runtime capture mode.
     */
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

    int frame_count = 0;

    cam.set_callback(
        [&](cv::Mat&& frame, auto /*ts*/)
        {
            frame_count++;

            /*
             * Print approximately once per second at a 60 FPS capture rate.
             * The callback itself remains lightweight so it does not disturb
             * the camera timing being measured.
             */
            if (frame_count % 60 == 0)
            {
                const auto& js =
                    cam.jitter_stats();

                std::printf(
                    "frame#%d size=%dx%d | "
                    "jitter mean=%.2fms stddev=%.3fms\n",
                    frame_count,
                    frame.cols,
                    frame.rows,
                    js.mean,
                    js.stddev()
                );
            }
        }
    );

    std::printf(
        "Camera test is running. Press Ctrl+C to stop.\n"
    );

    while (!g_stop.load())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)
        );
    }

    cam.close();

    const auto& js =
        cam.jitter_stats();

    std::printf(
        "\n== Final results ==\n"
        "Frames: %d | jitter mean=%.2fms stddev=%.3fms\n",
        frame_count,
        js.mean,
        js.stddev()
    );

    return 0;
}
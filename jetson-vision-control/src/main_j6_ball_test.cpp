/**
 * @file    main_j6_ball_test.cpp
 * @brief   Camera and ball-detection integration test.
 *
 * Starts the camera capture and ball detection tasks, then periodically
 * displays the latest ball position, velocity, and detection state.
 *
 * This test isolates the vision path from CAN and control processing.
 */

#include "task_camera_capture.hpp"
#include "task_ball_detect.hpp"
#include "system_state.hpp"

#include <csignal>
#include <atomic>
#include <cstdio>
#include <thread>
#include <chrono>

static std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a controlled shutdown request so the vision tasks can
 * release camera and processing resources before the process exits.
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
     * Use the same camera configuration expected by the normal vision
     * pipeline so this test exercises the production capture path.
     */
    if (!cam.start(
        1280,
        720,
        60,
        /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Khong the start camera\n"
        );

        return 1;
    }

    BallDetectorConfig cfg;

    /*
     * Detect the white ball against the dark table surface using a fixed
     * grayscale threshold.
     */
    cfg.white_threshold =
        200.0;

    /*
     * Keep the ROI disabled for this standalone test. Set a positive radius
     * together with roi_center_px when the physical table boundary needs to
     * be excluded from detection.
     */
    cfg.roi_radius_px =
        0.0f;

    /*
     * Reject small contours that are unlikely to represent the ball.
     *
     * This threshold is part of the detector configuration and can be tuned
     * independently without changing the test application structure.
     */
    cfg.min_area_px =
        2500.0;

    /*
     * Enable detector diagnostics so contour area and shape metrics can be
     * observed while selecting appropriate detection thresholds.
     */
    cfg.debug_print =
        true;

    TaskBallDetect ball;

    if (!ball.start(
        cam,
        cfg))
    {
        std::fprintf(
            stderr,
            "Khong the start ball detect\n"
        );

        cam.stop();

        return 1;
    }

    std::printf(
        "Dang chay. Ctrl+C de dung.\n"
    );

    while (!g_stop.load())
    {
        int16_t x;
        int16_t y;
        int16_t vx;
        int16_t vy;

        uint8_t detected;

        ball_state_read(
            system_state().ball,
            x,
            y,
            vx,
            vy,
            detected
        );

        /*
         * Print at 5 Hz so the output remains readable while the detector
         * continues processing camera frames at its own rate.
         */
        std::printf(
            "Ballx=%d Bally=%d vBallx=%d vBally=%d detected=%d\n",
            x,
            y,
            vx,
            vy,
            detected
        );

        std::this_thread::sleep_for(
            std::chrono::milliseconds(200)
        );
    }

    /*
     * Stop the detector before the camera so no processing thread can
     * access camera data during camera shutdown.
     */
    ball.stop();
    cam.stop();

    return 0;
}
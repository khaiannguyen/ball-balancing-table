/**
 * @file    intrinsic_calib_tool.cpp
 * @brief   Camera intrinsic calibration utility.
 *
 * Captures checkerboard observations and estimates the camera matrix and
 * distortion coefficients using OpenCV calibrateCamera().
 *
 * The resulting calibration is stored in calib/intrinsics.yaml and is used
 * by the extrinsic calibration stage.
 *
 * Calibration quality depends on capturing the checkerboard at different
 * positions and viewing angles, including near the image boundaries.
 */

#include "task_camera_capture.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>
#include <vector>

static const cv::Size PATTERN_SIZE(
    7,
    5
);

static const double SQUARE_SIZE_MM =
24.25;

static const int MIN_IMAGES =
10;

int main()
{
    TaskCameraCapture cam;

    if (!cam.start(
        1280,
        720,
        60,
        /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Failed to start camera.\n"
        );

        return 1;
    }

    std::atomic<bool> capture_requested{
        false
    };

    std::atomic<bool> quit_requested{
        false
    };

    /*
     * Read terminal commands asynchronously so camera capture and preview
     * continue running while the user waits for input.
     */
    std::thread input_thread(
        [&]()
        {
            std::string line;

            while (
                std::getline(
                    std::cin,
                    line))
            {
                if (line == "c")
                {
                    capture_requested.store(
                        true
                    );
                }
                else if (line == "q")
                {
                    quit_requested.store(
                        true
                    );

                    break;
                }
                else
                {
                    std::printf(
                        "Invalid command. "
                        "Enter 'c' to capture or 'q' to finish.\n"
                    );
                }
            }
        }
    );

    std::vector<
        std::vector<cv::Point2f>
    > image_points_all;

    cv::Size image_size;

    auto last_preview_save =
        std::chrono::steady_clock::now();

    std::printf(
        "Ready. Enter 'c' when the checkerboard is at a new "
        "position or viewing angle. Enter 'q' when enough images "
        "have been captured (minimum %d, recommended 15-20).\n",
        MIN_IMAGES
    );

    cv::Mat frame;

    std::chrono::steady_clock::time_point ts;

    while (!quit_requested.load())
    {
        if (cam.get_latest_frame(
            frame,
            ts))
        {
            image_size =
                frame.size();

            /*
             * Periodically save a preview so calibration can be performed
             * through an SSH/headless session.
             */
            auto now =
                std::chrono::steady_clock::now();

            if (
                std::chrono::duration<double>(
                    now - last_preview_save
                    ).count() > 0.5)
            {
                cv::imwrite(
                    "/tmp/intrinsic_calib_preview.jpg",
                    frame
                );

                last_preview_save =
                    now;
            }

            if (
                capture_requested.exchange(
                    false))
            {
                cv::Mat gray;

                cv::cvtColor(
                    frame,
                    gray,
                    cv::COLOR_BGR2GRAY
                );

                std::vector<cv::Point2f> corners;

                bool found =
                    cv::findChessboardCorners(
                        gray,
                        PATTERN_SIZE,
                        corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH |
                        cv::CALIB_CB_NORMALIZE_IMAGE
                    );

                if (found)
                {
                    cv::cornerSubPix(
                        gray,
                        corners,
                        cv::Size(11, 11),
                        cv::Size(-1, -1),
                        cv::TermCriteria(
                            cv::TermCriteria::EPS |
                            cv::TermCriteria::COUNT,
                            30,
                            0.001
                        )
                    );

                    image_points_all.push_back(
                        corners
                    );

                    /*
                     * Save the detected checkerboard overlay so individual
                     * calibration samples can be inspected later.
                     */
                    cv::Mat annotated =
                        frame.clone();

                    cv::drawChessboardCorners(
                        annotated,
                        PATTERN_SIZE,
                        corners,
                        found
                    );

                    char fname[128];

                    std::snprintf(
                        fname,
                        sizeof(fname),
                        "/tmp/intrinsic_calib_%02zu.jpg",
                        image_points_all.size()
                    );

                    cv::imwrite(
                        fname,
                        annotated
                    );

                    std::printf(
                        "Captured image #%zu (%s) - %s\n",
                        image_points_all.size(),
                        fname,
                        image_points_all.size() >=
                        static_cast<size_t>(MIN_IMAGES)
                        ? "minimum reached; 'q' may be entered"
                        : "minimum not reached"
                    );
                }
                else
                {
                    std::printf(
                        "Checkerboard not detected. "
                        "Ensure the board is fully visible, well lit, "
                        "and unobstructed, then try 'c' again.\n"
                    );
                }
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(30)
        );
    }

    cam.stop();

    if (input_thread.joinable())
    {
        input_thread.join();
    }

    if (
        image_points_all.size() <
        static_cast<size_t>(MIN_IMAGES))
    {
        std::fprintf(
            stderr,
            "Only %zu images were captured; at least %d are required. "
            "Run the tool again and capture more samples.\n",
            image_points_all.size(),
            MIN_IMAGES
        );

        return 1;
    }

    /*
     * Intrinsic calibration uses an arbitrary checkerboard coordinate system.
     * The robot origin and Servo1 direction are not relevant at this stage.
     */
    std::vector<cv::Point3f>
        object_points_template;

    for (
        int j = 0;
        j < PATTERN_SIZE.height;
        ++j)
    {
        for (
            int i = 0;
            i < PATTERN_SIZE.width;
            ++i)
        {
            object_points_template.emplace_back(
                i * SQUARE_SIZE_MM,
                j * SQUARE_SIZE_MM,
                0.0f
            );
        }
    }

    std::vector<
        std::vector<cv::Point3f>
    > object_points_all(
        image_points_all.size(),
        object_points_template
    );

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    double rms =
        cv::calibrateCamera(
            object_points_all,
            image_points_all,
            image_size,
            camera_matrix,
            dist_coeffs,
            rvecs,
            tvecs
        );

    std::printf(
        "\n=== calibrateCamera results ===\n"
    );

    std::printf(
        "Valid images: %zu\n",
        image_points_all.size()
    );

    std::printf(
        "RMS reprojection error: %.4f px (%s)\n",
        rms,
        rms < 0.5
        ? "good"
        : (
            rms < 1.0
            ? "acceptable"
            : "high - capture more samples at varied angles"
            )
    );

    std::cout
        << "Camera matrix K =\n"
        << camera_matrix
        << "\n";

    std::cout
        << "Distortion coefficients =\n"
        << dist_coeffs
        << "\n";

    cv::FileStorage fs(
        "calib/intrinsics.yaml",
        cv::FileStorage::WRITE
    );

    fs
        << "image_width"
        << image_size.width;

    fs
        << "image_height"
        << image_size.height;

    fs
        << "camera_matrix"
        << camera_matrix;

    fs
        << "dist_coeffs"
        << dist_coeffs;

    fs
        << "rms_reprojection_error"
        << rms;

    fs
        << "num_images"
        << static_cast<int>(
            image_points_all.size()
            );

    fs.release();

    std::printf(
        "\nSaved calib/intrinsics.yaml. "
        "Use this file for extrinsic calibration.\n"
    );

    return 0;
}
/**
 * @file    sanity_check_tool.cpp
 * @brief   Camera calibration and robot-frame sanity check utility.
 *
 * Loads the intrinsic and extrinsic calibration files, captures a camera
 * frame with a coordinate grid, and converts manually selected image pixels
 * into positions on the robot table plane.
 *
 * The calculation follows:
 *
 *   pixel
 *     -> undistorted camera ray
 *     -> robot-frame ray
 *     -> intersection with Z = 0
 *     -> robot-frame X/Y position
 *
 * The calculated position can then be compared with the manually measured
 * position of the corresponding servo axis.
 *
 * Interactive commands:
 *
 *   c     - capture a new frame and save the coordinate grid
 *   u v   - enter an image pixel coordinate
 *   q     - exit
 */

#include "task_camera_capture.hpp"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

int main()
{
    cv::FileStorage fs_intr(
        "calib/intrinsics.yaml",
        cv::FileStorage::READ
    );

    cv::FileStorage fs_extr(
        "calib/extrinsic.yaml",
        cv::FileStorage::READ
    );

    if (
        !fs_intr.isOpened() ||
        !fs_extr.isOpened())
    {
        std::fprintf(
            stderr,
            "Failed to open calib/intrinsics.yaml or "
            "calib/extrinsic.yaml. "
            "Run the intrinsic and extrinsic calibration tools first "
            "from the project working directory.\n"
        );

        return 1;
    }

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rvec;
    cv::Mat tvec;

    fs_intr["camera_matrix"] >>
        camera_matrix;

    fs_intr["dist_coeffs"] >>
        dist_coeffs;

    fs_extr["rvec"] >>
        rvec;

    fs_extr["tvec"] >>
        tvec;

    fs_intr.release();
    fs_extr.release();

    /*
     * solvePnP() returns the transform from the robot/world frame to the
     * camera frame. The inverse transform gives the camera position and
     * ray direction in the robot frame.
     */
    cv::Mat R;

    cv::Rodrigues(
        rvec,
        R
    );

    cv::Mat Rinv =
        R.t();

    cv::Mat cam_pos_world =
        -Rinv * tvec;

    double cam_z =
        cam_pos_world.at<double>(
            2,
            0
            );

    std::printf(
        "Camera position in the robot frame (mm): "
        "X=%.1f Y=%.1f Z=%.1f\n",
        cam_pos_world.at<double>(0, 0),
        cam_pos_world.at<double>(1, 0),
        cam_z
    );

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

    std::printf(
        "\nReady. Place the marker at the servo axis to be measured "
        "and enter 'c' to capture.\n"
        "Enter 'q' to exit.\n"
    );

    cv::Mat frame;

    std::chrono::steady_clock::time_point ts;

    bool have_frame = false;

    cv::Size frame_size;

    std::string line;

    while (std::getline(
        std::cin,
        line))
    {
        if (line == "q")
        {
            break;
        }

        if (line == "c")
        {
            /*
             * Capture the latest available frame. Allow up to approximately
             * one second for a frame to become available.
             */
            have_frame = false;

            for (int i = 0; i < 30; ++i)
            {
                if (cam.get_latest_frame(
                    frame,
                    ts))
                {
                    have_frame = true;
                    break;
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(33)
                );
            }

            if (!have_frame)
            {
                std::printf(
                    "No camera frame is available. "
                    "Enter 'c' again.\n"
                );

                continue;
            }

            frame_size =
                frame.size();

            cv::Mat grid =
                frame.clone();

            /*
             * Draw a 50-pixel grid with coordinate labels so the user can
             * estimate the pixel location of the physical marker.
             */
            for (
                int x = 0;
                x < grid.cols;
                x += 50)
            {
                cv::line(
                    grid,
                    { x, 0 },
                    { x, grid.rows },
                    cv::Scalar(
                        0,
                        255,
                        255
                    ),
                    1
                );

                if (x % 100 == 0)
                {
                    cv::putText(
                        grid,
                        std::to_string(x),
                        { x + 2, 15 },
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.35,
                        cv::Scalar(
                            0,
                            255,
                            255
                        ),
                        1
                    );
                }
            }

            for (
                int y = 0;
                y < grid.rows;
                y += 50)
            {
                cv::line(
                    grid,
                    { 0, y },
                    { grid.cols, y },
                    cv::Scalar(
                        0,
                        255,
                        255
                    ),
                    1
                );

                if (y % 100 == 0)
                {
                    cv::putText(
                        grid,
                        std::to_string(y),
                        { 2, y + 12 },
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.35,
                        cv::Scalar(
                            0,
                            255,
                            255
                        ),
                        1
                    );
                }
            }

            cv::imwrite(
                "/tmp/sanity_check_grid.jpg",
                grid
            );

            std::printf(
                "Saved /tmp/sanity_check_grid.jpg. "
                "Read the marker pixel coordinates (u v) from the grid "
                "and enter them, for example: 640 400.\n"
            );

            continue;
        }

        if (!have_frame)
        {
            std::printf(
                "No image has been captured yet. "
                "Enter 'c' first.\n"
            );

            continue;
        }

        std::istringstream iss(
            line
        );

        double u;
        double v;

        if (!(iss >> u >> v))
        {
            std::printf(
                "Invalid input. Enter 'u v' "
                "(for example: 640 400), "
                "'c' to capture, or 'q' to exit.\n"
            );

            continue;
        }

        if (
            u < 0 ||
            u >= frame_size.width ||
            v < 0 ||
            v >= frame_size.height)
        {
            std::printf(
                "Pixel coordinates (%.0f, %.0f) are outside "
                "the %dx%d image.\n",
                u,
                v,
                frame_size.width,
                frame_size.height
            );

            continue;
        }

        /*
         * Convert the distorted pixel into a normalized camera-frame ray.
         * The normalized ray uses z = 1.
         */
        std::vector<cv::Point2d> pixel_pts{
            {u, v}
        };

        std::vector<cv::Point2d> norm_pts;

        cv::undistortPoints(
            pixel_pts,
            norm_pts,
            camera_matrix,
            dist_coeffs
        );

        cv::Mat ray_cam =
            (cv::Mat_<double>(3, 1)
                << norm_pts[0].x,
                norm_pts[0].y,
                1.0
                );

        /*
         * Transform the ray into the robot frame and intersect it with
         * the table plane Z = 0.
         *
         * P_world =
         *     cam_pos_world +
         *     s * (Rinv * ray_cam)
         */
        cv::Mat ray_world =
            Rinv * ray_cam;

        double ray_z =
            ray_world.at<double>(
                2,
                0
                );

        if (std::abs(ray_z) < 1e-9)
        {
            std::printf(
                "The ray is parallel to the table plane. "
                "Unable to compute an intersection; "
                "check the selected pixel coordinate.\n"
            );

            continue;
        }

        double s =
            -cam_z / ray_z;

        double X =
            cam_pos_world.at<double>(0, 0) +
            s * ray_world.at<double>(0, 0);

        double Y =
            cam_pos_world.at<double>(1, 0) +
            s * ray_world.at<double>(1, 0);

        double dist_to_O =
            std::sqrt(
                X * X +
                Y * Y
            );

        std::printf(
            "Pixel (%.0f, %.0f) -> "
            "X=%.2f mm Y=%.2f mm | "
            "distance from O=%.2f mm\n"
            "Compare this result with the manually measured "
            "distance from O to the corresponding servo axis.\n",
            u,
            v,
            X,
            Y,
            dist_to_O
        );
    }

    cam.stop();

    return 0;
}
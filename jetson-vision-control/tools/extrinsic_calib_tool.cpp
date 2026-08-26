/**
 * @file    extrinsic_calib_tool.cpp
 * @brief   Camera-to-robot extrinsic calibration utility.
 *
 * Loads the camera intrinsics, captures a checkerboard placed at the
 * calibration origin, estimates the camera pose with solvePnP(), and
 * provides an interactive axis review before saving the result.
 *
 * Interactive commands:
 *
 *   c  - capture the current frame and solve the checkerboard pose
 *   fx - flip the robot X axis and recompute the pose
 *   fy - flip the robot Y axis and recompute the pose
 *   ok - save the reviewed calibration
 *   r  - discard the current frame and return to live capture
 *   q  - exit without saving
 */

#include "task_camera_capture.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

static const cv::Size PATTERN_SIZE(7, 5);

static const double SQUARE_SIZE_MM = 24.25;

static const double BOARD_LONG_MM =
8 * SQUARE_SIZE_MM;

static const double BOARD_SHORT_MM =
6 * SQUARE_SIZE_MM;

/*
 * Build checkerboard object points in the robot coordinate frame.
 *
 * The robot origin is located at the checkerboard center on the table plane
 * (Z = 0). Point ordering must match the raster order returned by
 * findChessboardCorners(): width changes fastest, followed by height.
 */
static std::vector<cv::Point3f> build_object_points(
    bool flip_x,
    bool flip_y)
{
    std::vector<cv::Point3f> pts;

    pts.reserve(
        PATTERN_SIZE.width *
        PATTERN_SIZE.height
    );

    for (int j = 0; j < PATTERN_SIZE.height; ++j)
    {
        for (int i = 0; i < PATTERN_SIZE.width; ++i)
        {
            double x_edge =
                (i + 1) * SQUARE_SIZE_MM;

            double y_edge =
                (j + 1) * SQUARE_SIZE_MM;

            double X =
                x_edge -
                BOARD_LONG_MM / 2.0;

            double Y =
                y_edge -
                BOARD_SHORT_MM / 2.0;

            if (flip_x)
            {
                X = -X;
            }

            if (flip_y)
            {
                Y = -Y;
            }

            pts.emplace_back(
                static_cast<float>(X),
                static_cast<float>(Y),
                0.0f
            );
        }
    }

    return pts;
}

int main()
{
    cv::FileStorage fs_in(
        "calib/intrinsics.yaml",
        cv::FileStorage::READ
    );

    if (!fs_in.isOpened())
    {
        std::fprintf(
            stderr,
            "Failed to open calib/intrinsics.yaml. "
            "Run this tool from the project working directory "
            "after running intrinsic_calib_tool.\n"
        );

        return 1;
    }

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;

    fs_in["camera_matrix"] >> camera_matrix;
    fs_in["dist_coeffs"] >> dist_coeffs;

    fs_in.release();

    std::cout
        << "Loaded camera intrinsics.\n"
        << "K =\n"
        << camera_matrix
        << "\n"
        << "dist =\n"
        << dist_coeffs
        << "\n";

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

    /*
     * Read terminal commands asynchronously so the live preview loop does
     * not block while waiting for user input.
     */
    std::mutex cmd_mtx;
    std::string pending_cmd;

    std::atomic<bool> quit_all{ false };

    std::thread input_thread(
        [&]()
        {
            std::string line;

            while (std::getline(
                std::cin,
                line))
            {
                {
                    std::lock_guard<std::mutex> lock(
                        cmd_mtx
                    );

                    pending_cmd = line;
                }

                if (line == "q")
                {
                    break;
                }
            }
        }
    );

    enum class State
    {
        LIVE,
        REVIEW
    };

    State state = State::LIVE;

    cv::Mat captured_frame;
    std::vector<cv::Point2f> captured_corners;

    bool flip_x = false;
    bool flip_y = false;

    cv::Mat rvec;
    cv::Mat tvec;

    auto last_preview_save =
        std::chrono::steady_clock::now();

    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;

    std::printf(
        "Ready. Place the checkerboard at the calibration position "
        "(origin centered, no shadows), then enter 'c' to detect it. "
        "Enter 'q' to exit at any time.\n"
    );

    while (!quit_all.load())
    {
        std::string cmd;

        {
            std::lock_guard<std::mutex> lock(
                cmd_mtx
            );

            cmd = pending_cmd;
            pending_cmd.clear();
        }

        if (cmd == "q")
        {
            quit_all.store(true);
            break;
        }

        if (state == State::LIVE)
        {
            if (cam.get_latest_frame(
                frame,
                ts))
            {
                auto now =
                    std::chrono::steady_clock::now();

                /*
                 * Save a periodic preview so the calibration can be performed
                 * through an SSH or headless session.
                 */
                if (
                    std::chrono::duration<double>(
                        now - last_preview_save
                        ).count() > 0.5)
                {
                    cv::imwrite(
                        "/tmp/extrinsic_calib_preview.jpg",
                        frame
                    );

                    last_preview_save = now;
                }
            }

            if (cmd == "c")
            {
                if (frame.empty())
                {
                    std::printf(
                        "No camera frame is available yet. "
                        "Try again shortly.\n"
                    );
                }
                else
                {
                    cv::Mat gray;

                    cv::cvtColor(
                        frame,
                        gray,
                        cv::COLOR_BGR2GRAY
                    );

                    std::vector<cv::Point2f> corners;

                    /*
                     * Detect the checkerboard and refine the corner locations
                     * before passing them to solvePnP().
                     */
                    bool found =
                        cv::findChessboardCorners(
                            gray,
                            PATTERN_SIZE,
                            corners,
                            cv::CALIB_CB_ADAPTIVE_THRESH |
                            cv::CALIB_CB_NORMALIZE_IMAGE
                        );

                    if (!found)
                    {
                        std::printf(
                            "Checkerboard not detected. "
                            "Ensure the board is fully visible, "
                            "well illuminated, and unobstructed, "
                            "then enter 'c' again.\n"
                        );
                    }
                    else
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

                        captured_frame =
                            frame.clone();

                        captured_corners =
                            corners;

                        flip_x = false;
                        flip_y = false;

                        auto obj_pts =
                            build_object_points(
                                flip_x,
                                flip_y
                            );

                        cv::solvePnP(
                            obj_pts,
                            captured_corners,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec
                        );

                        cv::Mat annotated =
                            captured_frame.clone();

                        /*
                         * Mark the detected checkerboard corners and draw
                         * the estimated robot axes for visual verification.
                         */
                        for (const auto& pt :
                            captured_corners)
                        {
                            cv::circle(
                                annotated,
                                pt,
                                4,
                                cv::Scalar(
                                    255,
                                    255,
                                    255
                                ),
                                -1
                            );
                        }

                        cv::drawFrameAxes(
                            annotated,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec,
                            static_cast<float>(
                                BOARD_LONG_MM * 1.2
                                )
                        );

                        cv::imwrite(
                            "/tmp/extrinsic_calib_annotated.jpg",
                            annotated
                        );

                        std::printf(
                            "\nCheckerboard detected and solvePnP completed.\n"
                            "Review /tmp/extrinsic_calib_annotated.jpg.\n"
                            "  X axis should run along the long board edge.\n"
                            "  Y axis should point toward Servo1.\n"
                            "  Z axis should point upward from the table.\n"
                            "Commands: 'fx' flip X | 'fy' flip Y | "
                            "'ok' save | 'r' recapture | 'q' exit\n"
                            "flip_x=%d flip_y=%d\n",
                            flip_x,
                            flip_y
                        );

                        state = State::REVIEW;
                    }
                }
            }
        }
        else
        {
            /* State::REVIEW */

            if (
                cmd == "fx" ||
                cmd == "fy" ||
                cmd == "ok" ||
                cmd == "r")
            {
                if (cmd == "r")
                {
                    std::printf(
                        "Discarding the current frame. "
                        "Returning to live capture mode.\n"
                    );

                    state = State::LIVE;
                }
                else if (
                    cmd == "fx" ||
                    cmd == "fy")
                {
                    if (cmd == "fx")
                    {
                        flip_x = !flip_x;
                    }

                    if (cmd == "fy")
                    {
                        flip_y = !flip_y;
                    }

                    auto obj_pts =
                        build_object_points(
                            flip_x,
                            flip_y
                        );

                    /*
                     * Recompute the pose using the same captured image.
                     * No new camera frame is required when only an axis
                     * direction is being corrected.
                     */
                    cv::solvePnP(
                        obj_pts,
                        captured_corners,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec
                    );

                    cv::Mat annotated =
                        captured_frame.clone();

                    for (const auto& pt :
                        captured_corners)
                    {
                        cv::circle(
                            annotated,
                            pt,
                            4,
                            cv::Scalar(
                                255,
                                255,
                                255
                            ),
                            -1
                        );
                    }

                    cv::drawFrameAxes(
                        annotated,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        static_cast<float>(
                            BOARD_LONG_MM * 1.2
                            )
                    );

                    cv::imwrite(
                        "/tmp/extrinsic_calib_annotated.jpg",
                        annotated
                    );

                    std::printf(
                        "Pose recomputed. Review "
                        "/tmp/extrinsic_calib_annotated.jpg. "
                        "flip_x=%d flip_y=%d\n",
                        flip_x,
                        flip_y
                    );
                }
                else if (cmd == "ok")
                {
                    cv::Mat R;

                    cv::Rodrigues(
                        rvec,
                        R
                    );

                    cv::FileStorage fs_out(
                        "calib/extrinsic.yaml",
                        cv::FileStorage::WRITE
                    );

                    fs_out
                        << "square_size_mm"
                        << SQUARE_SIZE_MM;

                    fs_out
                        << "pattern_size_w"
                        << PATTERN_SIZE.width;

                    fs_out
                        << "pattern_size_h"
                        << PATTERN_SIZE.height;

                    fs_out
                        << "flip_x"
                        << flip_x;

                    fs_out
                        << "flip_y"
                        << flip_y;

                    fs_out
                        << "rvec"
                        << rvec;

                    fs_out
                        << "tvec"
                        << tvec;

                    fs_out
                        << "rotation_matrix"
                        << R;

                    fs_out.release();

                    std::printf(
                        "\nSaved calib/extrinsic.yaml.\n"
                        "tvec (camera position in the robot frame, mm) =\n"
                    );

                    std::cout
                        << tvec
                        << "\n";

                    std::printf(
                        "Sanity check: |tvec| should be approximately "
                        "equal to the manually measured camera height "
                        "above the table origin.\n"
                    );

                    quit_all.store(true);
                }
            }
            else if (!cmd.empty())
            {
                std::printf(
                    "Invalid command in REVIEW mode. "
                    "Use: fx | fy | ok | r | q\n"
                );
            }
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(30)
        );
    }

    cam.stop();

    if (input_thread.joinable())
    {
        /*
         * The input thread may still be blocked inside getline(). If the
         * program exits through "q", the thread terminates normally.
         * If another command such as "ok" requests shutdown, detach the
         * thread so the application does not block waiting for another line.
         */
        input_thread.detach();
    }

    return 0;
}
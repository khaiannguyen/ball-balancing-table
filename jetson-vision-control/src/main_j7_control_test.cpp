/**
 * @file    main_j7_control_test.cpp
 * @brief   Full vision, control, and CAN integration test.
 *
 * Starts the camera, ball detector, control loop, CAN RX, and CAN TX tasks
 * as one integrated Jetson runtime.
 *
 * The application also records a low-rate diagnostic CSV containing the
 * principal control-loop inputs, outputs, and STM32 feedback.
 */

#include "task_camera_capture.hpp"
#include "task_ball_detect.hpp"
#include "task_control_loop.hpp"
#include "task_can_rx.hpp"
#include "task_can_tx.hpp"
#include "system_state.hpp"

#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <ctime>

std::atomic<bool> g_stop{ false };

/*
 * Convert SIGINT into a coordinated shutdown request so all runtime tasks
 * can be stopped explicitly in main().
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

    TaskCameraCapture cam;
    TaskBallDetect ball_detect;
    TaskControlLoop control_loop;
    TaskCanRx can_rx;
    TaskCanTx can_tx;

    /*
     * Start the camera first because all downstream vision processing depends
     * on a valid frame source.
     */
    if (!cam.start(
        1280,
        720,
        60,
        /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc camera\n"
        );

        return 1;
    }

    BallDetectorConfig cfg;

    cfg.white_threshold =
        200.0;

    cfg.roi_radius_px =
        0.0f;

    /*
     * Use the tuned minimum contour area required by this integration test
     * to reject small non-ball objects.
     */
    cfg.min_area_px =
        1500.0;

    cfg.debug_print =
        false;

    /*
     * Restrict detection to the calibrated table region so dark table
     * borders and surrounding hardware are excluded from the candidate set.
     */
    cfg.roi_center_px =
        cv::Point2f(
            635.0f,
            350.0f
        );

    cfg.roi_radius_px =
        350.0f;

    /*
     * The detector requires the camera calibration files to transform image
     * measurements into the physical table coordinate system.
     */
    if (!ball_detect.start(
        cam,
        cfg,
        "/home/khaian/balance_ball/calib"))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc ball detect\n"
        );

        cam.stop();

        return 1;
    }

    /*
     * Start CAN RX before CAN TX so STM32 telemetry reception is available
     * before Jetson begins transmitting control-related data.
     */
    if (!can_rx.start("can0"))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc CAN RX\n"
        );

        ball_detect.stop();
        cam.stop();

        return 1;
    }

    if (!can_tx.start("can0"))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc CAN TX\n"
        );

        can_rx.stop();
        ball_detect.stop();
        cam.stop();

        return 1;
    }

    /*
     * Start the control loop only after both vision and CAN communication
     * services are available.
     *
     * These gains and the output limit are part of this specific integration
     * test configuration and are intentionally kept unchanged here.
     */
    if (!control_loop.start(
        /*kp=*/0.02f,
        /*ki=*/0.0f,
        /*kd=*/0.01f,
        /*out_limit_deg=*/2.0f))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc control loop\n"
        );

        can_tx.stop();
        can_rx.stop();
        ball_detect.stop();
        cam.stop();

        return 1;
    }

    /*
     * Record the principal runtime signals at the same rate as the
     * diagnostic monitoring loop.
     *
     * The absolute path makes the test independent of the directory from
     * which the executable is launched.
     */
    const std::string csv_path =
        "/home/khaian/balance_ball/scripts/data.csv";

    std::ofstream csv(
        csv_path,
        std::ios::out |
        std::ios::trunc
    );

    if (!csv.is_open())
    {
        std::fprintf(
            stderr,
            "Canh bao: khong mo duoc %s de ghi log CSV, "
            "tiep tuc chay nhung KHONG ghi file.\n",
            csv_path.c_str()
        );
    }
    else
    {
        /*
         * Keep the CSV column order stable so recorded datasets can be
         * consumed by the existing offline analysis tools.
         */
        csv
            << "timestamp_ms,"
            << "S1,S2,S3,"
            << "roll_imu,pitch_imu,"
            << "roll_d,pitch_d,"
            << "Ballx,Bally,detected\n";

        csv.flush();

        std::printf(
            "J7: dang ghi log vao %s\n",
            csv_path.c_str()
        );
    }

    std::printf(
        "J7: he thong dang chay "
        "(camera + CAN + PID). Nhan Ctrl+C de dung.\n"
    );

    /*
     * Use the same monotonic clock as the runtime tasks so the CSV timestamp
     * is not affected by wall-clock adjustments.
     */
    auto t_start =
        std::chrono::steady_clock::now();

    while (!g_stop.load())
    {
        /*
         * The monitor runs at 5 Hz. This rate is sufficient for diagnostic
         * logging while leaving the real-time tasks independent from the
         * monitoring workload.
         */
        std::this_thread::sleep_for(
            std::chrono::milliseconds(200)
        );

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

        float roll_d;
        float pitch_d;

        attitude_desired_read(
            system_state().attitude_desired,
            roll_d,
            pitch_d
        );

        int32_t s1;
        int32_t s2;
        int32_t s3;

        telemetry_servo_read(
            system_state().servo,
            s1,
            s2,
            s3
        );

        float roll_imu;
        float pitch_imu;
        float vroll_imu;
        float vpitch_imu;
        float height_imu;

        telemetry_attitude_read(
            system_state().attitude,
            roll_imu,
            pitch_imu,
            vroll_imu,
            vpitch_imu,
            height_imu
        );

        bool stm32_ok =
            stm32_state_is_ok();

        /*
         * Print the key closed-loop signals so control behavior can be
         * inspected directly during the integration test.
         */
        std::printf(
            "[J7] ball=(%d,%d)mm det=%d  "
            "roll_d=%.4f pitch_d=%.4f deg  "
            "S1=%d S2=%d S3=%d  "
            "roll_imu=%.2f pitch_imu=%.2f  "
            "stm32_ok=%d\n",
            x,
            y,
            detected,
            roll_d,
            pitch_d,
            s1,
            s2,
            s3,
            roll_imu,
            pitch_imu,
            stm32_ok ? 1 : 0
        );

        /*
         * Store one diagnostic sample per monitoring cycle using elapsed
         * monotonic time relative to application startup.
         */
        if (csv.is_open())
        {
            auto now =
                std::chrono::steady_clock::now();

            double t_ms =
                std::chrono::duration<double, std::milli>(
                    now - t_start
                    ).count();

            csv
                << t_ms << ","
                << s1 << ","
                << s2 << ","
                << s3 << ","
                << roll_imu << ","
                << pitch_imu << ","
                << roll_d << ","
                << pitch_d << ","
                << x << ","
                << y << ","
                << (int)detected
                << "\n";

            /*
             * Flush each record so the diagnostic data remains available
             * even when the test is terminated with Ctrl+C.
             */
            csv.flush();
        }
    }

    /*
     * Stop downstream control/communication tasks before shutting down the
     * vision source they depend on.
     */
    control_loop.stop();
    can_tx.stop();
    can_rx.stop();
    ball_detect.stop();
    cam.stop();

    if (csv.is_open())
    {
        csv.close();

        std::printf(
            "J7: da dong file log %s\n",
            csv_path.c_str()
        );
    }

    std::printf(
        "J7: da dung toan bo he thong an toan.\n"
    );

    return 0;
}
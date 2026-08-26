/**
 * @file    main.cpp
 * @brief   Jetson vision and control application entry point.
 *
 * Initializes the camera, vision pipeline, CAN communication, control loop,
 * watchdog, video recording, and telemetry logging.
 *
 * The application uses a shared system-state layer to exchange measurements
 * and commands between concurrent runtime tasks.
 */

#include "task_camera_capture.hpp"
#include "task_ball_detect.hpp"
#include "task_control_loop.hpp"
#include "task_can_rx.hpp"
#include "task_can_tx.hpp"
#include "task_watchdog.hpp"
#include "task_video_record.hpp"
#include "system_state.hpp"

#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>

std::atomic<bool> g_stop{ false };

void on_sigint(int)
{
    g_stop.store(true);
}

/*
 * Runtime threads are created by their corresponding task classes.
 *
 * The control-loop scheduling policy is owned by TaskControlLoop so its
 * timing configuration remains close to the control implementation.
 */

int main()
{
    std::signal(SIGINT, on_sigint);

    TaskCameraCapture cam;
    TaskBallDetect    ball_detect;
    TaskControlLoop   control_loop;
    TaskCanRx         can_rx;
    TaskCanTx         can_tx;
    TaskWatchdog      watchdog;
    TaskVideoRecord   video_record;

    /*
     * Start the camera before any consumer task.
     *
     * Ball detection and video recording both consume frames from the
     * camera pipeline, so the producer must be available first.
     */
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0))
    {
        std::fprintf(
            stderr,
            "Loi: khong start duoc camera\n"
        );

        return 1;
    }

    BallDetectorConfig cfg;

    /*
     * Detection thresholds are calibrated for the current camera,
     * illumination, and ball geometry.
     *
     * These values are part of the vision configuration and should be
     * changed only when the corresponding camera/detection calibration
     * changes.
     */
    cfg.white_threshold = 230.0;
    cfg.min_area_px = 1500.0;
    cfg.min_circularity = 0.65;
    cfg.debug_print = false;

    /*
     * Restrict detection to the calibrated table region.
     *
     * The ROI reduces false detections from the surrounding mechanical
     * structure and background while keeping the complete usable ball
     * workspace inside the processing region.
     */
    cfg.roi_center_px = cv::Point2f(
        629.8f,
        362.4f
    );

    cfg.roi_radius_px = 400.0f;

    /*
     * Ball detection consumes frames from the camera pipeline and publishes
     * measurements to the shared system state.
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
     * CAN RX and TX share the same SocketCAN interface but have independent
     * responsibilities:
     *
     *   RX -> receive STM32 telemetry and status
     *   TX -> transmit commands and control outputs
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
     * The controller consumes the latest vision and telemetry state and
     * produces the desired platform attitude for transmission to STM32.
     *
     * Controller gains and output limits are application-level parameters.
     */
    if (!control_loop.start(
        /*kp=*/0.045f,
        /*ki=*/0.018f,
        /*kd=*/0.03f,
        /*out_limit_deg=*/3.5f))
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
     * Use one monotonic reference timestamp for runtime telemetry,
     * video synchronization, and CSV logging.
     *
     * A monotonic clock avoids discontinuities caused by wall-clock
     * adjustments during a running experiment.
     */
    auto t_start =
        std::chrono::steady_clock::now();

    /*
     * The watchdog supervises the runtime tasks independently from the
     * application logging loop.
     */
    watchdog.start(
        /*check_period_ms=*/1000
    );

    /*
     * Video recording is started after camera initialization because it is
     * another independent consumer of the camera frame stream.
     *
     * Recording failure is non-fatal: the control application can continue
     * operating without video logging.
     */
    {
        auto now_tp =
            std::chrono::system_clock::now();

        std::time_t now_c =
            std::chrono::system_clock::to_time_t(now_tp);

        char time_buf[32];

        std::strftime(
            time_buf,
            sizeof(time_buf),
            "%Y%m%d_%H%M%S",
            std::localtime(&now_c)
        );

        const std::string video_path =
            std::string(
                "/home/khaian/balance_ball/scripts/video_"
            ) +
            time_buf +
            ".mp4";

        const std::string video_ts_csv_path =
            std::string(
                "/home/khaian/balance_ball/scripts/video_timestamps_"
            ) +
            time_buf +
            ".csv";

        if (!video_record.start(
            cam,
            video_path,
            video_ts_csv_path,
            t_start,
            /*fps_hint=*/60.0))
        {
            std::fprintf(
                stderr,
                "Canh bao: khong start duoc TaskVideoRecord, "
                "tiep tuc chay nhung KHONG ghi video.\n"
            );
        }
    }

    /*
     * Store runtime telemetry in a fixed absolute path so experiment data
     * does not depend on the directory from which the application is
     * launched.
     */
    const std::string csv_path =
        "/home/khaian/balance_ball/scripts/data.csv";

    std::ofstream csv(
        csv_path,
        std::ios::out | std::ios::trunc
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
        csv <<
            "timestamp_ms,S1,S2,S3,roll_imu,pitch_imu,"
            "roll_d,pitch_d,height_d,Ballx,Bally,detected\n";

        csv.flush();

        std::printf(
            "main: dang ghi log vao %s\n",
            csv_path.c_str()
        );
    }

    std::printf(
        "main: he thong day du dang chay "
        "(camera+CAN+PID+watchdog+video). "
        "Nhan Ctrl+C de dung.\n"
    );

    /*
     * The main thread performs low-rate telemetry monitoring and logging.
     *
     * Real-time processing remains inside the dedicated runtime tasks.
     * This prevents diagnostic output and file I/O from blocking the
     * camera, control, or CAN processing paths.
     */
    while (!g_stop.load())
    {
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
        float height_d;

        attitude_desired_read(
            system_state().attitude_desired,
            roll_d,
            pitch_d,
            height_d
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

        std::printf(
            "[main] ball=(%d,%d)mm det=%d  "
            "roll_d=%.4f pitch_d=%.4f deg  "
            "height_d=%.2f mm  "
            "S1=%d S2=%d S3=%d  "
            "roll_imu=%.2f pitch_imu=%.2f  "
            "stm32_ok=%d\n",
            x,
            y,
            detected,
            roll_d,
            pitch_d,
            height_d,
            s1,
            s2,
            s3,
            roll_imu,
            pitch_imu,
            stm32_ok ? 1 : 0
        );

        /*
         * Flush every telemetry record so an unexpected process termination
         * does not leave the latest experiment samples only in userspace
         * buffers.
         */
        if (csv.is_open())
        {
            auto now =
                std::chrono::steady_clock::now();

            double t_ms =
                std::chrono::duration<double, std::milli>(
                    now - t_start
                    ).count();

            csv <<
                t_ms << "," <<
                s1 << "," <<
                s2 << "," <<
                s3 << "," <<
                roll_imu << "," <<
                pitch_imu << "," <<
                roll_d << "," <<
                pitch_d << "," <<
                height_d << "," <<
                x << "," <<
                y << "," <<
                (int)detected <<
                "\n";

            csv.flush();
        }
    }

    /*
     * Stop consumers before their shared producers.
     *
     * Video recording and ball detection must release their camera
     * dependencies before the camera itself is stopped.
     */
    watchdog.stop();
    control_loop.stop();
    can_tx.stop();
    can_rx.stop();

    video_record.stop();
    ball_detect.stop();
    cam.stop();

    if (csv.is_open())
    {
        csv.close();

        std::printf(
            "main: da dong file log %s\n",
            csv_path.c_str()
        );
    }

    std::printf(
        "main: da dung toan bo he thong an toan.\n"
    );

    return 0;
}
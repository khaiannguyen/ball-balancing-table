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
#include <fstream>     // THEM: ghi file CSV
#include <ctime>       // THEM: timestamp cho ten file / dong log

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

int main() {
    std::signal(SIGINT, on_sigint);

    TaskCameraCapture cam;
    TaskBallDetect    ball_detect;
    TaskControlLoop   control_loop;
    TaskCanRx         can_rx;
    TaskCanTx         can_tx;

    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Loi: khong start duoc camera\n");
        return 1;
    }

    BallDetectorConfig cfg;
    cfg.white_threshold = 200.0;
    cfg.roi_radius_px = 0.0f;
    cfg.min_area_px = 1500.0;   // da ha tu 2500.0 sau khi chan doan flicker
    cfg.debug_print = false;

    cfg.roi_center_px = cv::Point2f(635.0f, 350.0f);
    cfg.roi_radius_px = 350.0f;   // hoi nho hon ban kinh ban that mot chut de
                               // chac chan khong lot vien den/khung kim loai

    if (!ball_detect.start(cam, cfg, "/home/khaian/balance_ball/calib")) {
        std::fprintf(stderr, "Loi: khong start duoc ball detect\n");
        cam.stop();
        return 1;
    }

    if (!can_rx.start("can0")) {
        std::fprintf(stderr, "Loi: khong start duoc CAN RX\n");
        ball_detect.stop();
        cam.stop();
        return 1;
    }

    if (!can_tx.start("can0")) {
        std::fprintf(stderr, "Loi: khong start duoc CAN TX\n");
        can_rx.stop();
        ball_detect.stop();
        cam.stop();
        return 1;
    }

    if (!control_loop.start(/*kp=*/0.02f, /*ki=*/0.0f, /*kd=*/0.01f,
                             /*out_limit_deg=*/2.0f)) {
        std::fprintf(stderr, "Loi: khong start duoc control loop\n");
        can_tx.stop();
        can_rx.stop();
        ball_detect.stop();
        cam.stop();
        return 1;
    }

    /* ---- THEM: mo file CSV log o ~/balance_ball/scripts/data.csv ----
     * Duong dan TUYET DOI de khong phu thuoc thu muc dang chay chuong
     * trinh (giong bai hoc rut ra tu loi calib/intrinsics.yaml truoc day). */
    const std::string csv_path = "/home/khaian/balance_ball/scripts/data.csv";
    std::ofstream csv(csv_path, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::fprintf(stderr, "Canh bao: khong mo duoc %s de ghi log CSV, "
                     "tiep tuc chay nhung KHONG ghi file.\n", csv_path.c_str());
    } else {
        csv << "timestamp_ms,S1,S2,S3,roll_imu,pitch_imu,roll_d,pitch_d,Ballx,Bally,detected\n";
        csv.flush();
        std::printf("J7: dang ghi log vao %s\n", csv_path.c_str());
    }

    std::printf("J7: he thong dang chay (camera + CAN + PID). Nhan Ctrl+C de dung.\n");

    auto t_start = std::chrono::steady_clock::now();

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        int16_t x, y, vx, vy;
        uint8_t detected;
        ball_state_read(system_state().ball, x, y, vx, vy, detected);

        float roll_d, pitch_d;
        attitude_desired_read(system_state().attitude_desired, roll_d, pitch_d);

        int32_t s1, s2, s3;
        telemetry_servo_read(system_state().servo, s1, s2, s3);

        float roll_imu, pitch_imu, vroll_imu, vpitch_imu, height_imu;
        telemetry_attitude_read(system_state().attitude, roll_imu, pitch_imu,
                                 vroll_imu, vpitch_imu, height_imu);

        bool stm32_ok = stm32_state_is_ok();

        std::printf("[J7] ball=(%d,%d)mm det=%d  roll_d=%.4f pitch_d=%.4f deg  "
                    "S1=%d S2=%d S3=%d  roll_imu=%.2f pitch_imu=%.2f  stm32_ok=%d\n",
                    x, y, detected, roll_d, pitch_d, s1, s2, s3,
                    roll_imu, pitch_imu, stm32_ok ? 1 : 0);

        /* ---- Ghi 1 dong CSV moi chu ky giam sat (5Hz, khop tan so vong
         * lap nay) ---- */
        if (csv.is_open()) {
            auto now = std::chrono::steady_clock::now();
            double t_ms = std::chrono::duration<double, std::milli>(now - t_start).count();

            csv << t_ms << ","
                << s1 << "," << s2 << "," << s3 << ","
                << roll_imu << "," << pitch_imu << ","
                << roll_d << "," << pitch_d << ","
                << x << "," << y << "," << (int)detected << "\n";
            csv.flush();   // flush moi dong de khong mat du lieu neu Ctrl+C dot ngot
        }
    }

    control_loop.stop();
    can_tx.stop();
    can_rx.stop();
    ball_detect.stop();
    cam.stop();

    if (csv.is_open()) {
        csv.close();
        std::printf("J7: da dong file log %s\n", csv_path.c_str());
    }

    std::printf("J7: da dung toan bo he thong an toan.\n");
    return 0;
}
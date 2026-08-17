#include "task_camera_capture.hpp"
#include "task_ball_detect.hpp"
#include "task_control_loop.hpp"
#include "task_can_rx.hpp"
#include "task_can_tx.hpp"
#include "task_watchdog.hpp"
#include "task_video_record.hpp"   // THEM: ghi video (khong overlay, ve offline sau)
#include "system_state.hpp"
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>
#include <ctime>     // THEM: strftime/localtime cho ten file video co timestamp
#include <fstream>   // THEM (tu J7): ghi file CSV

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

/* THEM: nang priority SCHED_FIFO cho thread control loop - logic thuc su
 * nam o dau ham TaskControlLoop::run() (task_control_loop.cpp), khong goi
 * tu day vi std::thread khong the set scheduling param tu ben ngoai sau
 * khi thread da start mot cach an toan/don gian. */

int main() {
    std::signal(SIGINT, on_sigint);

    TaskCameraCapture cam;
    TaskBallDetect    ball_detect;
    TaskControlLoop   control_loop;
    TaskCanRx         can_rx;
    TaskCanTx         can_tx;
    TaskWatchdog      watchdog;
    TaskVideoRecord   video_record;   // THEM: ghi video song song, khong overlay
/**/
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Loi: khong start duoc camera\n");
        return 1;
    }

    BallDetectorConfig cfg;
    // 200 qua cao voi anh toi (bong bi cat con r~24px, duoi min_radius_px,
    // va circularity tut xuong ~0.39) -> ha ve 120 (da kiem chung tren
    // test_1280x720.jpg cho circularity ~0.71, r~42px, dung vi tri bong).
    cfg.white_threshold = 230.0;
    cfg.min_area_px = 1500.0;   // da ha tu 2500.0 sau khi chan doan flicker (J7)
    // Bong o sat mep ban thuong bi chieu sang khong deu (nua toi do nen
    // ban toi, nua loa sang do nen ngoai ban) -> hull khong tron hoan
    // toan du la bong that. Do thuc te tren test_1280x720.jpg: circularity
    // ~0.70-0.71 voi bong that o mep ban. Ha nguong tu 0.75 (mac dinh
    // trong ball_detector.hpp) xuong 0.65 de khong loai nham truong hop nay.
    cfg.min_circularity = 0.65;
    cfg.debug_print = false;

    // ROI: gioi han vung detect trong pham vi mat ban tron, loai bo nen/vat
    // phan chieu ngoai vien khoi threshold (J7). TODO: do chinh xac lai
    // (Huong D, J8) - hien tai la gia tri uoc luong.
    cfg.roi_center_px = cv::Point2f(629.8f, 362.4f);
    cfg.roi_radius_px = 400.0f; // hoi nho hon ban kinh ban that de chac
                                        // chan khong lot vien den/khung kim loai

    if (!ball_detect.start(cam, cfg, "/home/khaian/balance_ball/calib")) {
        std::fprintf(stderr, "Loi: khong start duoc ball detect\n");
        cam.stop();
        return 1;
    }

    if (!can_rx.start("can0")) {
        std::fprintf(stderr, "Loi: khong start duoc CAN RX\n");
        ball_detect.stop(); cam.stop();
        return 1;
    }

    if (!can_tx.start("can0")) {
        std::fprintf(stderr, "Loi: khong start duoc CAN TX\n");
        can_rx.stop(); ball_detect.stop(); cam.stop();
        return 1;
    }

    
    //  if (!control_loop.start(/*kp=*/0.0625f, /*ki=*/0.028f, /*kd=*/0.04125f,/*out_limit_deg=*/3.5f)) 
    //  if (!control_loop.start(/*kp=*/0.055f, /*ki=*/0.018f, /*kd=*/0.035f,/*out_limit_deg=*/3.5f)) {
    if (!control_loop.start(/*kp=*/0.045f, /*ki=*/0.018f, /*kd=*/0.03f,/*out_limit_deg=*/3.5f)) {
        std::fprintf(stderr, "Loi: khong start duoc control loop\n");
        can_tx.stop(); can_rx.stop(); ball_detect.stop(); cam.stop();
        return 1;
    }

    // SUA: t_start doi len SOM HON (truoc ca csv/video) - dung LAM MOC
    // THOI GIAN CHUNG cho CA data.csv LAN video_timestamps.csv, de 2 file
    // khop duoc voi nhau chinh xac theo timestamp_ms khi ve lai offline
    // sau nay (video KHONG con phu thuoc fps danh nghia de dong bo nua).
    auto t_start = std::chrono::steady_clock::now();

    watchdog.start(/*check_period_ms=*/1000);

    // THEM: start ghi video - PHAI sau khi cam.start() thanh cong (da co
    // o tren), KHONG can doi ball_detect/control_loop vi day la consumer
    // DOC LAP hoan toan voi 2 cai do, chi doc them frame giong ball_detect
    // dang doc (FrameBox ho tro nhieu reader dong thoi, xem
    // task_camera_capture.hpp). Duong dan cung thu muc voi data.csv, dat
    // ten co timestamp de KHONG ghi de lan chay truoc (khac data.csv hien
    // dang trunc/ghi de moi lan chay - video nang hon, mat cong quay lai
    // nen uu tien khong mat du lieu cu).
    {
        auto now_tp = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now_tp);
        char time_buf[32];
        std::strftime(time_buf, sizeof(time_buf), "%Y%m%d_%H%M%S", std::localtime(&now_c));

        const std::string video_path = std::string(
            "/home/khaian/balance_ball/scripts/video_") + time_buf + ".mp4";
        const std::string video_ts_csv_path = std::string(
            "/home/khaian/balance_ball/scripts/video_timestamps_") + time_buf + ".csv";

        if (!video_record.start(cam, video_path, video_ts_csv_path, t_start,
                                 /*fps_hint=*/60.0)) {
            std::fprintf(stderr, "Canh bao: khong start duoc TaskVideoRecord, "
                         "tiep tuc chay nhung KHONG ghi video.\n");
        }
    }

    /* ---- THEM (tu J7): mo file CSV log o ~/balance_ball/scripts/data.csv
     * Duong dan TUYET DOI de khong phu thuoc thu muc dang chay chuong
     * trinh (bai hoc rut ra tu loi calib/intrinsics.yaml o J7). */
    const std::string csv_path = "/home/khaian/balance_ball/scripts/data.csv";
    std::ofstream csv(csv_path, std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
        std::fprintf(stderr, "Canh bao: khong mo duoc %s de ghi log CSV, "
                     "tiep tuc chay nhung KHONG ghi file.\n", csv_path.c_str());
    } else {
        csv << "timestamp_ms,S1,S2,S3,roll_imu,pitch_imu,roll_d,pitch_d,height_d,Ballx,Bally,detected\n";
        csv.flush();
        std::printf("main: dang ghi log vao %s\n", csv_path.c_str());
    }

    std::printf("main: he thong day du dang chay (camera+CAN+PID+watchdog+video). "
                "Nhan Ctrl+C de dung.\n");

    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   // 5Hz, khop tan so log J7

        int16_t x, y, vx, vy;
        uint8_t detected;
        ball_state_read(system_state().ball, x, y, vx, vy, detected);

        float roll_d, pitch_d, height_d;
        attitude_desired_read(system_state().attitude_desired, roll_d, pitch_d, height_d);

        int32_t s1, s2, s3;
        telemetry_servo_read(system_state().servo, s1, s2, s3);

        float roll_imu, pitch_imu, vroll_imu, vpitch_imu, height_imu;
        telemetry_attitude_read(system_state().attitude, roll_imu, pitch_imu,
                                 vroll_imu, vpitch_imu, height_imu);

        bool stm32_ok = stm32_state_is_ok();

        std::printf("[main] ball=(%d,%d)mm det=%d  roll_d=%.4f pitch_d=%.4f deg  height_d=%.2f mm  "
                    "S1=%d S2=%d S3=%d  roll_imu=%.2f pitch_imu=%.2f  stm32_ok=%d\n",
                    x, y, detected, roll_d, pitch_d, height_d, s1, s2, s3,
                    roll_imu, pitch_imu, stm32_ok ? 1 : 0);

        // TODO: log RSS dinh ky neu muon tu dong theo doi memory leak
        // (doc /proc/self/status truong VmRSS) thay vi chi xem htop tay.

        /* ---- Ghi 1 dong CSV moi chu ky giam sat (5Hz) ---- */
        if (csv.is_open()) {
            auto now = std::chrono::steady_clock::now();
            double t_ms = std::chrono::duration<double, std::milli>(now - t_start).count();

            csv << t_ms << ","
                << s1 << "," << s2 << "," << s3 << ","
                << roll_imu << "," << pitch_imu << ","
                << roll_d << "," << pitch_d << "," << height_d << ","
                << x << "," << y << "," << (int)detected << "\n";
            csv.flush();   // flush moi dong de khong mat du lieu neu Ctrl+C dot ngot
        }
    }

    watchdog.stop();
    control_loop.stop();
    can_tx.stop();
    can_rx.stop();
    video_record.stop();   // THEM: dung TRUOC cam.stop() vi phu thuoc cam (giong ball_detect)
    ball_detect.stop();
    cam.stop();

    if (csv.is_open()) {
        csv.close();
        std::printf("main: da dong file log %s\n", csv_path.c_str());
    }

    std::printf("main: da dung toan bo he thong an toan.\n");
    return 0;
}
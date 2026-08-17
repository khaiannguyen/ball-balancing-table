#include "task_ball_detect.hpp"
#include "system_state.hpp"
#include "task_watchdog.hpp"
#include <cstdio>
#include <chrono>
#include <cmath>   // THEM: std::lround

bool TaskBallDetect::start(TaskCameraCapture& cam, BallDetectorConfig cfg,
                            const std::string& calib_dir) {
    if (running_.load(std::memory_order_relaxed)) {
        std::fprintf(stderr, "TaskBallDetect: da chay roi\n");
        return false;
    }

    std::string intr_path = calib_dir + "/intrinsics.yaml";
    std::string extr_path = calib_dir + "/extrinsic.yaml";

    cv::FileStorage fs_intr(intr_path, cv::FileStorage::READ);
    cv::FileStorage fs_extr(extr_path, cv::FileStorage::READ);
    if (!fs_intr.isOpened() || !fs_extr.isOpened()) {
        std::fprintf(stderr,
                      "TaskBallDetect: khong doc duoc %s hoac %s — kiem tra "
                      "duong dan calib_dir da truyen dung chua.\n",
                      intr_path.c_str(), extr_path.c_str());
        return false;
    }
    cv::Mat camera_matrix, dist_coeffs, rvec, tvec;
    fs_intr["camera_matrix"] >> camera_matrix;
    fs_intr["dist_coeffs"] >> dist_coeffs;
    fs_extr["rvec"] >> rvec;
    fs_extr["tvec"] >> tvec;
    fs_intr.release();
    fs_extr.release();

    detector_ = std::make_unique<BallDetector>(camera_matrix, dist_coeffs, rvec, tvec, cfg);
    cam_ = &cam;

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::make_unique<std::thread>(&TaskBallDetect::run, this);
    std::printf("TaskBallDetect: da start\n");
    return true;
}

void TaskBallDetect::stop() {
    if (!running_.load(std::memory_order_relaxed)) return;
    running_.store(false, std::memory_order_relaxed);
    if (thread_ && thread_->joinable()) thread_->join();
    std::printf("TaskBallDetect: da dung.\n");
}

void TaskBallDetect::run() {
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    std::chrono::steady_clock::time_point last_ts{};
    bool has_last_ts = false;

    double last_x_mm = 0.0, last_y_mm = 0.0;
    bool has_last_pos = false;

    // THEM: gia tri van toc DA LOC (EMA), duoc giu qua cac vong lap de
    // lam muot dan thay vi nhay theo tung sai phan tho. Khoi tao 0.
    float vx_filt = 0.f, vy_filt = 0.f;
    bool  has_filt = false; // lan dau tien co van toc thi khong loc (tranh
                            // lag khoi dong tu 0), tu lan thu 2 tro di moi EMA

    // THEM (P1 - debounce): dem so frame LIEN TIEP khong thay bong. Chi
    // bao detected=0 sau khi mat bong ON DINH qua MISS_THRESHOLD frame,
    // tranh 1 frame nhieu/loa sang thoang qua lam STM32/PID phan ung
    // nham (flicker det=1/0 lien tuc da quan sat thuc te o J8).
    int miss_count = 0;
    constexpr int MISS_THRESHOLD = 5;

    while (running_.load(std::memory_order_relaxed)) {
        task_alive_mark(ALIVE_BIT_BALL_DETECT);   // THEM: bao song cho TaskWatchdog moi vong lap

        if (!cam_->get_latest_frame(frame, ts)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Bỏ qua nếu chưa có frame mới hơn lần xử lý trước (tránh xử lý
        // trùng cùng 1 frame nhiều lần khi vòng lặp nhanh hơn camera).
        if (has_last_ts && ts == last_ts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        double x_mm, y_mm;
        bool found = detector_->detect(frame, x_mm, y_mm);

        if (found) {
            miss_count = 0;   // THEM: reset ngay khi thay bong lai

            if (has_last_pos && has_last_ts) {
                double dt = std::chrono::duration<double>(ts - last_ts).count();

                // Chi tinh van toc moi neu dt "hop le": khong qua nho (tranh
                // chia cho ~0 gay gia tri vo cung lon) VA khong qua lon (vd
                // sau 1 khoang mat bong dai — sai phan qua nhieu frame se
                // KHONG dai dien cho toc do tuc thoi thuc, ma la trung binh
                // "nhay coc" tren quang duong dai => giu nguyen bo loc cu,
                // khong cap nhat, tranh xung dot gia tri.
                if (dt > 1e-4 && dt <= max_valid_dt_s_) {
                    float vx_raw = (float)((x_mm - last_x_mm) / dt);
                    float vy_raw = (float)((y_mm - last_y_mm) / dt);

                    if (!has_filt) {
                        // SUA: truoc day gan THANG vx_filt = vx_raw (khong
                        // loc gi) cho mau dau tien sau khi bat lai bong -
                        // neu vi tri bong bi nhieu detect ngay luc bat lai
                        // (hay xay ra o mep ROI, xem ghi chu circularity/
                        // lighting trong main.cpp), vx_raw co the spike lon
                        // va di THANG vao D-term PID khong qua loc nao ca.
                        // Gio: khoi dong bo loc EMA tu 0 (vx_filt_=0), tuc
                        // mau dau tien = vel_alpha_ * vx_raw thay vi 100%
                        // vx_raw - giam bien do cu hich dau tien di he so
                        // vel_alpha_, van bat kip chuyen dong that trong
                        // vai frame ke tiep qua EMA binh thuong.
                        vx_filt = vel_alpha_ * vx_raw;
                        vy_filt = vel_alpha_ * vy_raw;
                        has_filt = true;
                    } else {
                        // Loc thong thap kieu EMA (exponential moving
                        // average): gia tri moi = alpha*raw + (1-alpha)*cu.
                        // alpha nho -> muot hon nhung tre theo chuyen dong
                        // that nhieu hon; alpha=1 -> tuong duong khong loc.
                        vx_filt = vel_alpha_ * vx_raw + (1.f - vel_alpha_) * vx_filt;
                        vy_filt = vel_alpha_ * vy_raw + (1.f - vel_alpha_) * vy_filt;
                    }
                }
                // else: dt ngoai khoang hop le -> GIU NGUYEN vx_filt/vy_filt
                // cu, khong cap nhat bang gia tri "nhay coc" bat thuong.
            }

            ball_state_write_pos(system_state().ball, (int16_t)x_mm, (int16_t)y_mm);
            // SUA: round-to-nearest thay vi truncate-toward-zero, tranh
            // thien lech he thong nho khi ep float ve int16_t.
            ball_state_write_vel(system_state().ball,
                                  (int16_t)std::lround(vx_filt),
                                  (int16_t)std::lround(vy_filt));
            ball_state_write_detected(system_state().ball, 1);

            last_x_mm = x_mm;
            last_y_mm = y_mm;
            has_last_pos = true;
        } else {
            miss_count++;   // THEM

            if (miss_count >= MISS_THRESHOLD) {
                // Mat bong ON DINH (du so frame lien tiep) — xoa het
                // pos/vel cu ve 0, bao detected=0. Tranh de gia tri cu
                // treo lai (nguy hiem cho task_can_tx/task_control_loop
                // neu vo tinh doc pos ma khong kiem tra co detected truoc).
                ball_state_write_pos(system_state().ball, 0, 0);
                ball_state_write_vel(system_state().ball, 0, 0);
                ball_state_write_detected(system_state().ball, 0);

                has_last_pos = false; // reset, tranh tinh van toc "nhay" khi bong xuat hien lai
                has_filt = false;     // THEM: reset bo loc EMA, tranh dung
                                      // van toc cu (truoc khi mat bong) tron
                                      // lan voi vi tri moi khi bong xuat hien lai
            }
            // else: mien_count < MISS_THRESHOLD -> COI LA MISS THOANG QUA,
            // GIU NGUYEN detected/pos/vel cu trong system_state(), khong
            // ghi de. TaskControlLoop se tiep tuc dung gia tri cu 1-2
            // chu ky nua, khong bi giat theo nhieu 1-frame.
        }

        last_ts = ts;
        has_last_ts = true;
    }
}
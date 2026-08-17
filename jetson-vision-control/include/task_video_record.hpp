#pragma once
#include "task_camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>
#include <string>
#include <chrono>

/* TaskVideoRecord — CHI ghi video + timestamp cua tung frame, KHONG ve gi
 * len frame ca (theo yeu cau: tach rieng ghi va ve, ve se lam offline sau
 * tren video da ghi). Consume frame TU DOC LAP voi TaskBallDetect, giong
 * het cach TaskBallDetect dang lam (poll get_latest_frame() qua FrameBox
 * double-buffer co san trong TaskCameraCapture) - KHONG dung gi den logic
 * ball detect/PID/CAN, chi doc them 1 nguon frame giong ball_detect dang
 * doc, hoan toan khong tranh chap (FrameBox ho tro nhieu reader cung luc).
 *
 * Ghi 2 file:
 *   1. video_path (vd .mp4)      - noi dung video tho, KHONG overlay.
 *   2. timestamps_csv_path       - 2 cot: frame_index, timestamp_ms. Cot
 *      timestamp_ms dung CHUNG mot moc t_start (steady_clock) voi
 *      data.csv (main.cpp truyen vao) - de sau nay ve lai co the KHOP
 *      chinh xac frame video voi dong du lieu PID/setpoint tuong ung theo
 *      thoi gian, khong can dua vao fps danh nghia cua video (fps thuc te
 *      co the jitter, xem JitterStats trong camera_pipeline.hpp). */
class TaskVideoRecord {
public:
    TaskVideoRecord() = default;
    ~TaskVideoRecord() { stop(); }

    TaskVideoRecord(const TaskVideoRecord&)            = delete;
    TaskVideoRecord& operator=(const TaskVideoRecord&) = delete;

    // cam phai da start() thanh cong truoc khi goi ham nay.
    // t_start: PHAI la CUNG 1 gia tri voi t_start dung de tinh timestamp_ms
    // trong data.csv (main.cpp) - de 2 file co chung goc thoi gian, khop
    // duoc voi nhau khi ve lai offline. fps_hint chi dung de ghi header
    // video (toc do phat danh nghia) - KHONG anh huong do chinh xac cua
    // timestamps_csv (cot do moi la nguon that de dong bo).
    bool start(TaskCameraCapture& cam,
               const std::string& video_path,
               const std::string& timestamps_csv_path,
               std::chrono::steady_clock::time_point t_start,
               double fps_hint = 30.0);
    void stop();

    bool is_running() const { return running_.load(std::memory_order_relaxed); }

private:
    void run();

    TaskCameraCapture* cam_ = nullptr;
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> running_{false};

    cv::VideoWriter writer_;
    std::ofstream   ts_csv_;
    std::string     video_path_;
    double          fps_hint_ = 30.0;
    std::chrono::steady_clock::time_point t_start_{};
};
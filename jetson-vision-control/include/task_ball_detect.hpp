#pragma once
#include "task_camera_capture.hpp"
#include "ball_detector.hpp"
#include <thread>
#include <atomic>
#include <memory>
#include <string>

/* TaskBallDetect — thread thật, consume frame từ TaskCameraCapture (J6
 * bước 0), chạy BallDetector, ghi kết quả vào system_state().ball — đúng
 * chỗ task_can_tx (J4) đang đọc giả lập. Chỉ thay nguồn ghi, không đụng gì
 * logic gửi CAN trong task_can_tx. */
class TaskBallDetect {
public:
    TaskBallDetect() = default;
    ~TaskBallDetect() { stop(); }

    TaskBallDetect(const TaskBallDetect&)            = delete;
    TaskBallDetect& operator=(const TaskBallDetect&) = delete;

    // cam phải đã start() thành công trước khi gọi hàm này.
    // Đọc calib/intrinsics.yaml + calib/extrinsic.yaml (đường dẫn tương
    // đối, chạy chương trình từ thư mục ~/balance_ball).
    //bool start(TaskCameraCapture& cam, BallDetectorConfig cfg = {});
    bool start(TaskCameraCapture& cam, BallDetectorConfig cfg = {},
           const std::string& calib_dir = "calib");
    void stop();

    bool is_running() const { return running_.load(std::memory_order_relaxed); }

    // THEM: he so loc EMA cho vx/vy (0 < alpha <= 1). alpha nho hon =
    // muot hon nhung tre (lag) nhieu hon. alpha = 1.0 nghia la KHONG loc
    // (giu hanh vi cu, sai phan tho). Goi truoc start() neu muon doi mac dinh.
    void set_velocity_filter_alpha(float alpha) { vel_alpha_ = alpha; }

    // THEM: dt toi da (giay) duoc coi la "hop le" de tinh van toc. Neu
    // khoang cach giua 2 lan thay bong lien tiep lon hon gia tri nay (vd
    // do mat bong lau roi xuat hien lai), BO QUA phep tinh van toc lan do
    // (giu nguyen vx/vy cu thay vi tao ra 1 gia tri "giat" bat thuong do
    // chia cho dt lon/nho khong dai dien cho chuyen dong that).
    void set_max_valid_dt(double max_dt_s) { max_valid_dt_s_ = max_dt_s; }

private:
    void run();

    TaskCameraCapture* cam_ = nullptr;
    std::unique_ptr<BallDetector> detector_;
    std::unique_ptr<std::thread>  thread_;
    std::atomic<bool> running_{false};

    float  vel_alpha_       = 0.35f;  // mac dinh: loc vua phai
    double max_valid_dt_s_  = 0.15;   // mac dinh: ~3 chu ky camera 60Hz lien tiep
};
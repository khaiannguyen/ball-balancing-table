#pragma once
#include "camera_pipeline.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

/* FrameBox — double buffer để consumer (task_ball_detect ở J6) lấy frame mới
 * nhất mà không chặn writer (camera callback thread) lâu.
 *
 * Nguyên tắc:
 *  - ready_idx  : chỉ số buffer đang "sẵn sàng để đọc" (0 hoặc 1, -1 = chưa
 *    có frame nào).
 *  - Writer luôn ghi vào buffer NGƯỢC với ready_idx hiện tại, rồi mới công
 *    bố ready_idx = buffer vừa ghi. Nhờ vậy writer không bao giờ ghi đè lên
 *    đúng buffer mà reader đang cầm đọc — kể cả khi reader đọc chậm hơn 1
 *    chu kỳ camera.
 *  - mtx[i] bảo vệ buffer i: writer phải lock được mtx[write_idx] mới ghi;
 *    reader phải lock được mtx[ready_idx] mới đọc. Nếu reader đang giữ khóa
 *    lâu bất thường (chậm hơn hẳn 16.6ms @60fps), writer sẽ tự chờ ở đây
 *    thay vì đâm vào dữ liệu đang bị đọc dở — đây là "mutex ngắn" nói ở
 *    file tiến độ, không phải để tránh race, mà để dồn/serialize đúng lúc
 *    có tranh chấp thực sự (hiếm khi xảy ra ở tải bình thường).
 *  - Không dùng seqlock cho cv::Mat vì struct cv::Mat có nhiều field
 *    (data/rows/cols/step) không atomic-safe nếu đọc/ghi đồng thời; mutex
 *    ngắn quanh lúc gán con trỏ là đủ.
 */
struct FrameBox {
    cv::Mat buffers[2];
    std::chrono::steady_clock::time_point timestamps[2];
    std::mutex mtx[2];
    std::atomic<int> ready_idx{-1};

    // Gọi từ camera callback thread (writer).
    void publish(cv::Mat&& frame, std::chrono::steady_clock::time_point ts) {
        int cur = ready_idx.load(std::memory_order_acquire);
        int write_idx = (cur == 0) ? 1 : 0; // nếu cur == -1 thì write_idx = 0

        {
            std::lock_guard<std::mutex> lk(mtx[write_idx]);
            buffers[write_idx] = std::move(frame); // gán con trỏ, chỉ tăng refcount
            timestamps[write_idx] = ts;
        }
        ready_idx.store(write_idx, std::memory_order_release);
    }

    // Gọi từ consumer thread (reader, vd task_ball_detect). Trả false nếu
    // chưa có frame nào. out được gán bằng operator= của cv::Mat (rẻ, chỉ
    // tăng refcount con trỏ pixel) — an toàn để dùng out sau khi hàm này
    // return vì writer không đụng lại buffer này cho tới lần publish kế.
    bool get_latest(cv::Mat& out, std::chrono::steady_clock::time_point& ts) {
        int idx = ready_idx.load(std::memory_order_acquire);
        if (idx < 0) return false;

        std::lock_guard<std::mutex> lk(mtx[idx]);
        out = buffers[idx];
        ts  = timestamps[idx];
        return true;
    }
};

/* TaskCameraCapture — thread thật của hệ thống chính, thay thế
 * camera_pipeline_test/camera_preview (2 tool độc lập của J5).
 * task_ball_detect (J6) sẽ là consumer đầu tiên qua get_latest_frame(). */
class TaskCameraCapture {
public:
    TaskCameraCapture() = default;
    ~TaskCameraCapture() { stop(); }

    TaskCameraCapture(const TaskCameraCapture&)            = delete;
    TaskCameraCapture& operator=(const TaskCameraCapture&) = delete;

    // Mở pipeline + start. Trả false nếu CameraPipeline::open() thất bại.
    bool start(int width, int height, int fps, int sensor_id = 0);

    // Dừng sạch: đóng pipeline, không cần thread riêng vì CameraPipeline
    // dùng callback trong chính GStreamer streaming thread (giống J5), nên
    // không có std::thread nào cần join() ở lớp này — chỉ cần close() đúng
    // thứ tự.
    void stop();

    bool is_running() const { return running_.load(std::memory_order_relaxed); }

    // API cho consumer (task_ball_detect). Trả false nếu chưa có frame nào.
    bool get_latest_frame(cv::Mat& out, std::chrono::steady_clock::time_point& ts) {
        return box_.get_latest(out, ts);
    }

    // Cho phép đọc jitter stats để log/debug giống camera_pipeline_test.
    const JitterStats& jitter_stats() const { return pipeline_.jitter_stats(); }

private:
    CameraPipeline pipeline_;
    FrameBox       box_;
    std::atomic<bool> running_{false};
};
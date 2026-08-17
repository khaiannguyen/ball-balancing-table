#include "task_camera_capture.hpp"
#include <csignal>
#include <atomic>
#include <cstdio>
#include <thread>
#include <chrono>

static std::atomic<bool> g_stop{false};
static void on_sigint(int) { g_stop.store(true); }

int main() {
    std::signal(SIGINT, on_sigint);

    TaskCameraCapture cam;
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Khong the start camera capture\n");
        return 1;
    }

    // Vòng lặp consumer giả lập — sau này chính là task_ball_detect.
    // Lấy frame mới nhất ~10Hz để in log, không cần đúng 60fps ở test này.
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    int got_count = 0, miss_count = 0;

    while (!g_stop.load()) {
        if (cam.get_latest_frame(frame, ts)) {
            got_count++;
            if (got_count % 30 == 0) { // in log mỗi ~3s ở 10Hz poll
                const auto& js = cam.jitter_stats();
                std::printf(
                    "[cam] got=%d miss=%d frame=%dx%d jitter mean=%.2fms stddev=%.2fms\n",
                    got_count, miss_count, frame.cols, frame.rows,
                    js.mean, js.stddev());
            }
        } else {
            miss_count++; // chưa có frame nào (thường chỉ xảy ra lúc mới start)
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::printf("Dung sach... got=%d miss=%d\n", got_count, miss_count);
    cam.stop();
    return 0;
}
#include "camera_pipeline.hpp"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

int main() {
    std::signal(SIGINT, on_sigint);

    CameraPipeline cam;
    if (!cam.open(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Mo camera that bai\n");
        return 1;
    }

    const bool has_display = (std::getenv("DISPLAY") != nullptr);
    if (!has_display) {
        std::printf("Khong co DISPLAY (chay SSH headless) - se ghi anh ra "
                     "/tmp/preview_latest.jpg moi ~0.5s, mo bang VS Code de xem.\n");
    }

    std::atomic<int> frame_idx{0};

    cam.set_callback([&](cv::Mat&& frame, auto /*ts*/) {
        // Vẽ crosshair tâm khung hình để canh tâm bàn qua mắt
        cv::Point center(frame.cols / 2, frame.rows / 2);
        cv::drawMarker(frame, center, cv::Scalar(0, 0, 255),
                        cv::MARKER_CROSS, 40, 2);
        cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);

        int idx = frame_idx.fetch_add(1);

        if (has_display) {
            cv::imshow("preview", frame);
            cv::waitKey(1);
        } else if (idx % 30 == 0) { // ghi ~2 anh/giay o 60fps
            cv::imwrite("/tmp/preview_latest.jpg", frame);
        }
    });

    std::printf("Dang chay preview. Ctrl+C de dung.\n");
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cam.close();
    std::printf("Da dung sach.\n");
    return 0;
}
#include "camera_pipeline.hpp"
#include <cstdio>
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

    int frame_count = 0;
    cam.set_callback([&](cv::Mat&& frame, auto /*ts*/) {
        frame_count++;
        if (frame_count % 60 == 0) { // in mỗi ~1 giây
            const auto& js = cam.jitter_stats();
            std::printf("frame#%d size=%dx%d | jitter mean=%.2fms stddev=%.3fms\n",
                        frame_count, frame.cols, frame.rows, js.mean, js.stddev());
        }
    });

    std::printf("Dang chay camera test. Ctrl+C de dung.\n");
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    cam.close();
    const auto& js = cam.jitter_stats();
    std::printf("\n== Ket qua cuoi ==\nSo frame: %d | jitter mean=%.2fms stddev=%.3fms\n",
                frame_count, js.mean, js.stddev());
    return 0;
}
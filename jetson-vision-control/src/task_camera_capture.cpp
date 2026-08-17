#include "task_camera_capture.hpp"
#include "task_watchdog.hpp"
#include <cstdio>

bool TaskCameraCapture::start(int width, int height, int fps, int sensor_id) {
    if (running_.load(std::memory_order_relaxed)) {
        std::fprintf(stderr, "TaskCameraCapture: da chay roi, goi start() 2 lan\n");
        return false;
    }

    // Dang ky callback TRUOC khi open() — new_sample co the duoc goi ngay
    // sau khi pipeline chuyen PLAYING, phai chac callback da san sang.
    pipeline_.set_callback(
        [this](cv::Mat&& frame, std::chrono::steady_clock::time_point ts) {
            task_alive_mark(ALIVE_BIT_CAMERA);   // THEM: mark moi khi CO FRAME THAT,
                                                  // dung noi camera thuc su "con song"
            box_.publish(std::move(frame), ts);
        });

    if (!pipeline_.open(width, height, fps, sensor_id)) {
        std::fprintf(stderr, "TaskCameraCapture: CameraPipeline::open() that bai\n");
        return false;
    }

    running_.store(true, std::memory_order_relaxed);
    std::printf("TaskCameraCapture: da start %dx%d@%dfps (sensor-id=%d)\n",
                width, height, fps, sensor_id);
    return true;   // XOA dong task_alive_mark() thua o day - khong con y nghia
                   // "moi vong lap" nua, doi cho callback lo
}

void TaskCameraCapture::stop() {
    if (!running_.load(std::memory_order_relaxed)) return;
    pipeline_.close();
    running_.store(false, std::memory_order_relaxed);
    std::printf("TaskCameraCapture: da dung.\n");
}
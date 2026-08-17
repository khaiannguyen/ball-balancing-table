#include "task_camera_capture.hpp"
#include "task_ball_detect.hpp"
#include "system_state.hpp"
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
        std::fprintf(stderr, "Khong the start camera\n");
        return 1;
    }

    // Ban tron mau den, bong trang — ROI mac dinh TAT (0). Neu thay nhieu
    // false-positive tu nen ngoai ban (tuong/vat sang), chinh
    // roi_center_px/roi_radius_px cho khop khung hinh thuc te roi build
    // lai — doc pixel tam+ban kinh ban tron qua /tmp/*.jpg cua cac tool
    // truoc (vd extrinsic_calib_annotated.jpg) de uoc luong.
    BallDetectorConfig cfg;
    cfg.white_threshold = 200.0;
    cfg.roi_radius_px = 0.0f; // doi > 0 va set roi_center_px neu can mask
    cfg.min_area_px = 2500.0; // uoc luong tu anh: den ~1100px^2, bong ~6000px^2
                               // BAT debug_print de do chinh xac roi chinh lai so nay
    cfg.debug_print = true;   // tam thoi BAT de xem area/circularity that —
                               // TAT lai (false) sau khi da chon duoc nguong dung

    TaskBallDetect ball;
    if (!ball.start(cam, cfg)) {
        std::fprintf(stderr, "Khong the start ball detect\n");
        cam.stop();
        return 1;
    }

    std::printf("Dang chay. Ctrl+C de dung.\n");

    while (!g_stop.load()) {
        int16_t x, y, vx, vy;
        uint8_t detected;
        ball_state_read(system_state().ball, x, y, vx, vy, detected);
        std::printf("Ballx=%d Bally=%d vBallx=%d vBally=%d detected=%d\n",
                    x, y, vx, vy, detected);
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5Hz de mat theo doi duoc
    }

    ball.stop();
    cam.stop();
    return 0;
}
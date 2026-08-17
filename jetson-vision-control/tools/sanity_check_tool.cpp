// tools/sanity_check_tool.cpp
//
// Sanity-check calib/extrinsic.yaml + calib/intrinsics.yaml: đặt vật đánh
// dấu (đầu bút, đồng xu...) tại tâm trục xoay mỗi servo, chụp ảnh, nhập
// tọa độ pixel của vật đó (đọc trên ảnh có lưới đã lưu), tool tính ra tọa
// độ mm trong hệ trục robot (gốc O) — so với số đo tay từ O đến servo.
//
// Toán học: pixel (u,v) -> undistort thành tia hướng trong hệ camera ->
// chuyển tia đó sang hệ world bằng R,t từ solvePnP -> tìm giao điểm tia
// với mặt phẳng Z=0 (mặt bàn) -> ra tọa độ (X,Y,0) mm trong hệ robot.
//
// Cách dùng:
//   ./sanity_check_tool
//   - Gõ "c" + Enter để chụp — ảnh lưu /tmp/sanity_check_grid.jpg, có lưới
//     50px kèm số tọa độ để bạn đọc gần đúng vị trí pixel từng bulong.
//   - Với mỗi servo, gõ "u v" (2 số cách nhau bởi khoảng trắng, đơn vị
//     pixel, đọc trên ảnh lưới) rồi Enter — tool in ra (X,Y) mm + khoảng
//     cách tới O, so sánh với số đo tay.
//   - Gõ "c" lại nếu muốn chụp ảnh khác, "q" để thoát.

#include "task_camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <thread>
#include <chrono>

int main() {
    cv::FileStorage fs_intr("calib/intrinsics.yaml", cv::FileStorage::READ);
    cv::FileStorage fs_extr("calib/extrinsic.yaml", cv::FileStorage::READ);
    if (!fs_intr.isOpened() || !fs_extr.isOpened()) {
        std::fprintf(stderr,
                      "Khong doc duoc calib/intrinsics.yaml hoac calib/extrinsic.yaml — "
                      "chay tu thu muc ~/balance_ball, da chay 2 tool calib truoc chua?\n");
        return 1;
    }
    cv::Mat camera_matrix, dist_coeffs, rvec, tvec;
    fs_intr["camera_matrix"] >> camera_matrix;
    fs_intr["dist_coeffs"] >> dist_coeffs;
    fs_extr["rvec"] >> rvec;
    fs_extr["tvec"] >> tvec;
    fs_intr.release();
    fs_extr.release();

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat Rinv = R.t();                 // R^T
    cv::Mat cam_pos_world = -Rinv * tvec;  // vi tri camera trong he world
    double cam_z = cam_pos_world.at<double>(2, 0);

    std::printf("Vi tri camera trong he robot (mm): X=%.1f Y=%.1f Z=%.1f\n",
                cam_pos_world.at<double>(0, 0),
                cam_pos_world.at<double>(1, 0), cam_z);

    TaskCameraCapture cam;
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Khong the start camera\n");
        return 1;
    }

    std::printf(
        "\nSan sang. Dat vat danh dau tai truc servo can do, go 'c'+Enter de chup.\n"
        "'q'+Enter de thoat.\n");

    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    bool have_frame = false;
    cv::Size frame_size;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line == "q") break;

        if (line == "c") {
            // Lấy frame mới nhất, đợi tối đa ~1s nếu chưa có.
            for (int i = 0; i < 30; ++i) {
                if (cam.get_latest_frame(frame, ts)) { have_frame = true; break; }
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
            }
            if (!have_frame) {
                std::printf("Chua co frame tu camera, thu 'c' lai.\n");
                continue;
            }
            frame_size = frame.size();

            cv::Mat grid = frame.clone();
            for (int x = 0; x < grid.cols; x += 50) {
                cv::line(grid, {x, 0}, {x, grid.rows}, cv::Scalar(0, 255, 255), 1);
                if (x % 100 == 0)
                    cv::putText(grid, std::to_string(x), {x + 2, 15},
                                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);
            }
            for (int y = 0; y < grid.rows; y += 50) {
                cv::line(grid, {0, y}, {grid.cols, y}, cv::Scalar(0, 255, 255), 1);
                if (y % 100 == 0)
                    cv::putText(grid, std::to_string(y), {2, y + 12},
                                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1);
            }
            cv::imwrite("/tmp/sanity_check_grid.jpg", grid);
            std::printf(
                "Da luu /tmp/sanity_check_grid.jpg — doc toa do pixel (u v) cua vat "
                "danh dau tren luoi, go 'u v' (vd: 640 400) roi Enter.\n");
            continue;
        }

        if (!have_frame) {
            std::printf("Chua chup anh nao, go 'c' truoc.\n");
            continue;
        }

        std::istringstream iss(line);
        double u, v;
        if (!(iss >> u >> v)) {
            std::printf("Dinh dang khong hop le. Go 'u v' (vd: 640 400), 'c' de chup, 'q' de thoat.\n");
            continue;
        }
        if (u < 0 || u >= frame_size.width || v < 0 || v >= frame_size.height) {
            std::printf("Toa do (%.0f, %.0f) nam ngoai anh %dx%d.\n",
                        u, v, frame_size.width, frame_size.height);
            continue;
        }

        // Undistort -> tia huong trong he camera (z=1 normalized).
        std::vector<cv::Point2d> pixel_pts{{u, v}};
        std::vector<cv::Point2d> norm_pts;
        cv::undistortPoints(pixel_pts, norm_pts, camera_matrix, dist_coeffs);
        cv::Mat ray_cam = (cv::Mat_<double>(3, 1) << norm_pts[0].x, norm_pts[0].y, 1.0);

        // Chuyen tia sang he world, tim giao voi mat phang Z=0:
        // P_world = cam_pos_world + s * (Rinv * ray_cam)
        cv::Mat ray_world = Rinv * ray_cam;
        double ray_z = ray_world.at<double>(2, 0);
        if (std::abs(ray_z) < 1e-9) {
            std::printf("Tia song song mat ban, khong tinh duoc — kiem tra lai toa do pixel.\n");
            continue;
        }
        double s = -cam_z / ray_z;
        double X = cam_pos_world.at<double>(0, 0) + s * ray_world.at<double>(0, 0);
        double Y = cam_pos_world.at<double>(1, 0) + s * ray_world.at<double>(1, 0);
        double dist_to_O = std::sqrt(X * X + Y * Y);

        std::printf(
            "Pixel (%.0f, %.0f) -> X=%.2f mm  Y=%.2f mm  |khoang cach toi O|=%.2f mm\n"
            "-> so voi so do tay tu O den servo nay, lech vai mm la dat.\n",
            u, v, X, Y, dist_to_O);
    }

    cam.stop();
    return 0;
}
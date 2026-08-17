// tools/intrinsic_calib_tool.cpp
//
// Hiệu chuẩn nội tại (intrinsic) camera — bước tiên quyết bắt buộc trước
// extrinsic_calib_tool (solvePnP cần camera matrix K + distCoeffs đúng, nếu
// không kết quả extrinsic sẽ sai lệch nhiều, không chỉ vài mm).
//
// Cách dùng (SSH headless, không cần DISPLAY):
//   ./intrinsic_calib_tool
//   - Xem preview tại /tmp/intrinsic_calib_preview.jpg (refresh ~0.5s/lần,
//     mở bằng VS Code hoặc image viewer qua SSHFS/scp).
//   - Gõ "c" + Enter ở terminal mỗi khi board đã ở vị trí/góc mới muốn chụp.
//   - Gõ "q" + Enter khi đã đủ ảnh (khuyến nghị 15-20 ảnh) để chạy calib.
//
// QUAN TRỌNG khi chụp: mỗi ảnh phải khác góc nghiêng / vị trí trong khung
// hình so với các ảnh trước (đặc biệt cần vài ảnh board áp sát 4 góc/rìa
// khung hình, không chỉ giữa khung) — calibrateCamera cần đủ đa dạng góc
// nhìn để ước lượng đúng distortion, nếu ảnh nào cũng board ở giữa/thẳng
// thì kết quả sẽ không đáng tin dù reprojection error thấp.

#include "task_camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <chrono>
#include <cstdio>

static const cv::Size PATTERN_SIZE(7, 5);   // 7x5 góc trong (board 8x6 ô)
static const double   SQUARE_SIZE_MM = 24.25;
static const int      MIN_IMAGES = 10;      // tối thiểu để chạy, khuyến nghị 15-20

int main() {
    TaskCameraCapture cam;
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Khong the start camera\n");
        return 1;
    }

    std::atomic<bool> capture_requested{false};
    std::atomic<bool> quit_requested{false};

    // Thread đọc lệnh từ terminal, không chặn vòng lặp preview/capture chính.
    std::thread input_thread([&]() {
        std::string line;
        while (std::getline(std::cin, line)) {
            if (line == "c") {
                capture_requested.store(true);
            } else if (line == "q") {
                quit_requested.store(true);
                break;
            } else {
                std::printf("Lenh khong hop le, go 'c' de chup hoac 'q' de ket thuc\n");
            }
        }
    });

    std::vector<std::vector<cv::Point2f>> image_points_all;
    cv::Size image_size;
    auto last_preview_save = std::chrono::steady_clock::now();

    std::printf(
        "San sang. Go 'c'+Enter de chup khi checkerboard o vi tri/goc moi, "
        "'q'+Enter khi da du anh (toi thieu %d, khuyen nghi 15-20).\n",
        MIN_IMAGES);

    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;

    while (!quit_requested.load()) {
        if (cam.get_latest_frame(frame, ts)) {
            image_size = frame.size();

            // Lưu preview định kỳ để xem qua SSH (giống camera_preview J5).
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_preview_save).count() > 0.5) {
                cv::imwrite("/tmp/intrinsic_calib_preview.jpg", frame);
                last_preview_save = now;
            }

            if (capture_requested.exchange(false)) {
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                std::vector<cv::Point2f> corners;
                bool found = cv::findChessboardCorners(
                    gray, PATTERN_SIZE, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

                if (found) {
                    cv::cornerSubPix(
                        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                          30, 0.001));
                    image_points_all.push_back(corners);

                    // Lưu ảnh + overlay góc detect được để soát lại sau nếu cần.
                    cv::Mat annotated = frame.clone();
                    cv::drawChessboardCorners(annotated, PATTERN_SIZE, corners, found);
                    char fname[128];
                    std::snprintf(fname, sizeof(fname),
                                  "/tmp/intrinsic_calib_%02zu.jpg",
                                  image_points_all.size());
                    cv::imwrite(fname, annotated);

                    std::printf("Da chup anh #%zu (luu %s) — %s\n",
                                image_points_all.size(), fname,
                                image_points_all.size() >= (size_t)MIN_IMAGES
                                    ? "du toi thieu, co the go 'q' bat cu luc nao"
                                    : "chua du toi thieu");
                } else {
                    std::printf(
                        "Khong thay checkerboard trong frame — kiem tra board co "
                        "trong khung, du sang, khong bi che, roi thu 'c' lai.\n");
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    cam.stop();
    if (input_thread.joinable()) input_thread.join();

    if (image_points_all.size() < (size_t)MIN_IMAGES) {
        std::fprintf(stderr,
                      "Chi co %zu anh, can toi thieu %d — chua chay calibrateCamera. "
                      "Chay lai tool va chup them.\n",
                      image_points_all.size(), MIN_IMAGES);
        return 1;
    }

    // Object points: lưới chuẩn (0,0)..(6*sq,4*sq) — KHÔNG cần canh theo O
    // hay hướng Servo1 ở bước này, vì đây chỉ là hiệu chuẩn nội tại của
    // riêng ống kính/cảm biến, không liên quan hệ trục robot.
    std::vector<cv::Point3f> object_points_template;
    for (int j = 0; j < PATTERN_SIZE.height; ++j) {
        for (int i = 0; i < PATTERN_SIZE.width; ++i) {
            object_points_template.emplace_back(
                i * SQUARE_SIZE_MM, j * SQUARE_SIZE_MM, 0.0f);
        }
    }
    std::vector<std::vector<cv::Point3f>> object_points_all(
        image_points_all.size(), object_points_template);

    cv::Mat camera_matrix, dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        object_points_all, image_points_all, image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs);

    std::printf("\n=== Ket qua calibrateCamera ===\n");
    std::printf("So anh dung: %zu\n", image_points_all.size());
    std::printf("RMS reprojection error: %.4f px  (%s)\n", rms,
                rms < 0.5 ? "tot" : (rms < 1.0 ? "chap nhan duoc" : "CAO, nen chup lai them anh da dang goc hon"));
    std::cout << "Camera matrix K =\n" << camera_matrix << "\n";
    std::cout << "Dist coeffs =\n" << dist_coeffs << "\n";

    cv::FileStorage fs("calib/intrinsics.yaml", cv::FileStorage::WRITE);
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    fs << "rms_reprojection_error" << rms;
    fs << "num_images" << (int)image_points_all.size();
    fs.release();

    std::printf("\nDa luu calib/intrinsics.yaml — dung file nay cho extrinsic_calib_tool.\n");
    return 0;
}
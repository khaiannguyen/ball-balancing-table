// tools/extrinsic_calib_tool.cpp
//
// Hiệu chuẩn ngoại tại (extrinsic) — cần chạy SAU intrinsic_calib_tool
// (đọc calib/intrinsics.yaml). Chụp live 1 khung hình checkerboard đặt
// đúng vị trí calib thật (tâm O trùng tâm checkerboard, không có bóng),
// chạy solvePnP, vẽ overlay 3 trục để bạn xác nhận bằng mắt trước khi lưu.
//
// Cách dùng (SSH headless):
//   ./extrinsic_calib_tool
//   - Xem preview tại /tmp/extrinsic_calib_preview.jpg
//   - Gõ "c" + Enter khi board đã đặt đúng vị trí calib (tâm O, không bóng,
//     đủ sáng, nhìn thấy trọn 4 góc) để thử detect + solvePnP.
//   - Sau khi detect thành công, xem ảnh overlay 3 trục tại
//     /tmp/extrinsic_calib_annotated.jpg — kiểm tra:
//       * Trục Y (xanh lá) phải chỉ về phía Servo1.
//       * Trục X (xanh dương) vuông góc Y, dọc cạnh dài board.
//       * Trục Z (đỏ) phải chỉ LÊN TRÊN (ra khỏi mặt bàn, hướng về camera),
//         không chỉ xuống dưới mặt bàn.
//   - Nếu trục nào sai chiều, gõ "fx" (lật X) hoặc "fy" (lật Y) rồi Enter —
//     chương trình tính lại NGAY (không cần chụp lại ảnh) và ghi đè overlay,
//     xem lại ảnh, lặp tới khi đúng cả 3 trục.
//   - Gõ "ok" + Enter để lưu calib/extrinsic.yaml.
//   - Gõ "r" + Enter để bỏ khung hình hiện tại, chụp lại từ đầu.
//   - Gõ "q" + Enter để thoát không lưu.

#include "task_camera_capture.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstdio>

static const cv::Size PATTERN_SIZE(7, 5);   // 7x5 góc trong (board 8x6 ô)
static const double   SQUARE_SIZE_MM = 24.25;
static const double   BOARD_LONG_MM  = 8 * SQUARE_SIZE_MM;  // 194.0mm — trục X
static const double   BOARD_SHORT_MM = 6 * SQUARE_SIZE_MM;  // 145.5mm — trục Y (O->Servo1)

// Object points trong hệ trục robot, gốc O = tâm checkerboard (Z=0, mặt bàn).
// Thứ tự phải khớp raster order của findChessboardCorners cho PATTERN_SIZE(7,5):
// duyệt theo width(7) nhanh, height(5) chậm.
static std::vector<cv::Point3f> build_object_points(bool flip_x, bool flip_y) {
    std::vector<cv::Point3f> pts;
    pts.reserve(PATTERN_SIZE.width * PATTERN_SIZE.height);
    for (int j = 0; j < PATTERN_SIZE.height; ++j) {
        for (int i = 0; i < PATTERN_SIZE.width; ++i) {
            double x_edge = (i + 1) * SQUARE_SIZE_MM;
            double y_edge = (j + 1) * SQUARE_SIZE_MM;
            double X = x_edge - BOARD_LONG_MM / 2.0;
            double Y = y_edge - BOARD_SHORT_MM / 2.0;
            if (flip_x) X = -X;
            if (flip_y) Y = -Y;
            pts.emplace_back((float)X, (float)Y, 0.0f);
        }
    }
    return pts;
}

int main() {
    cv::FileStorage fs_in("calib/intrinsics.yaml", cv::FileStorage::READ);
    if (!fs_in.isOpened()) {
        std::fprintf(stderr,
                      "Khong doc duoc calib/intrinsics.yaml — chay tu thu muc "
                      "~/balance_ball va da chay intrinsic_calib_tool truoc chua?\n");
        return 1;
    }
    cv::Mat camera_matrix, dist_coeffs;
    fs_in["camera_matrix"] >> camera_matrix;
    fs_in["dist_coeffs"] >> dist_coeffs;
    fs_in.release();
    std::cout << "Da doc intrinsics — K=\n" << camera_matrix
              << "\ndist=\n" << dist_coeffs << "\n";

    TaskCameraCapture cam;
    if (!cam.start(1280, 720, 60, /*sensor_id=*/0)) {
        std::fprintf(stderr, "Khong the start camera\n");
        return 1;
    }

    // ---- Input thread: đọc lệnh từ terminal, khong chan vong lap preview ----
    std::mutex cmd_mtx;
    std::string pending_cmd;
    std::atomic<bool> quit_all{false};
    std::thread input_thread([&]() {
        std::string line;
        while (std::getline(std::cin, line)) {
            {
                std::lock_guard<std::mutex> lk(cmd_mtx);
                pending_cmd = line;
            }
            if (line == "q") break;
        }
    });

    enum class State { LIVE, REVIEW };
    State state = State::LIVE;

    cv::Mat captured_frame;
    std::vector<cv::Point2f> captured_corners;
    bool flip_x = false, flip_y = false;
    cv::Mat rvec, tvec;

    auto last_preview_save = std::chrono::steady_clock::now();
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;

    std::printf(
        "San sang. Dat checkerboard dung vi tri calib (tam O, khong bong), "
        "go 'c'+Enter de thu detect. 'q'+Enter de thoat bat cu luc nao.\n");

    while (!quit_all.load()) {
        std::string cmd;
        {
            std::lock_guard<std::mutex> lk(cmd_mtx);
            cmd = pending_cmd;
            pending_cmd.clear();
        }

        if (cmd == "q") { quit_all.store(true); break; }

        if (state == State::LIVE) {
            if (cam.get_latest_frame(frame, ts)) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration<double>(now - last_preview_save).count() > 0.5) {
                    cv::imwrite("/tmp/extrinsic_calib_preview.jpg", frame);
                    last_preview_save = now;
                }
            }

            if (cmd == "c") {
                if (frame.empty()) {
                    std::printf("Chua co frame nao tu camera, thu lai sau 1 chut.\n");
                } else {
                    cv::Mat gray;
                    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
                    std::vector<cv::Point2f> corners;
                    bool found = cv::findChessboardCorners(
                        gray, PATTERN_SIZE, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

                    if (!found) {
                        std::printf(
                            "Khong thay checkerboard — kiem tra board trong khung, "
                            "du sang, khong bi che, roi 'c' lai.\n");
                    } else {
                        cv::cornerSubPix(
                            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                              30, 0.001));
                        captured_frame = frame.clone();
                        captured_corners = corners;
                        flip_x = false;
                        flip_y = false;

                        auto obj_pts = build_object_points(flip_x, flip_y);
                        cv::solvePnP(obj_pts, captured_corners, camera_matrix, dist_coeffs,
                                     rvec, tvec);

                        cv::Mat annotated = captured_frame.clone();
                        for (const auto& pt : captured_corners) {
                            cv::circle(annotated, pt, 4, cv::Scalar(255, 255, 255), -1);
                        }
                        cv::drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, tvec,
                                        (float)(BOARD_LONG_MM * 1.2)); // truc dai ~233mm, de nhin ro
                        cv::imwrite("/tmp/extrinsic_calib_annotated.jpg", annotated);

                        std::printf(
                            "\nDa detect + solvePnP. Xem /tmp/extrinsic_calib_annotated.jpg\n"
                            "  Truc X (do) co doc theo canh dai board khong?\n"
                            "  Truc Y (xanh la) co huong ve phia Servo1 khong?\n"
                            "  Truc Z (xanh duong) co chi LEN TREN — thuong se RAT NGAN tren anh\n"
                            "  vi camera nhin gan thang tu tren xuong, dung lo neu ngan, chi can\n"
                            "  khong bi dam nguoc xuong duoi mat ban.\n"
                            "Lenh: 'fx' lat X | 'fy' lat Y | 'ok' luu | 'r' chup lai | 'q' thoat\n"
                            "flip_x=%d flip_y=%d\n", flip_x, flip_y);
                        state = State::REVIEW;
                    }
                }
            }
        } else { // State::REVIEW
            if (cmd == "fx" || cmd == "fy" || cmd == "ok" || cmd == "r") {
                if (cmd == "r") {
                    std::printf("Bo qua khung hinh nay, quay lai che do chup live.\n");
                    state = State::LIVE;
                } else if (cmd == "fx" || cmd == "fy") {
                    if (cmd == "fx") flip_x = !flip_x;
                    if (cmd == "fy") flip_y = !flip_y;

                    auto obj_pts = build_object_points(flip_x, flip_y);
                    cv::solvePnP(obj_pts, captured_corners, camera_matrix, dist_coeffs,
                                 rvec, tvec);

                    cv::Mat annotated = captured_frame.clone();
                    for (const auto& pt : captured_corners) {
                        cv::circle(annotated, pt, 4, cv::Scalar(255, 255, 255), -1);
                    }
                    cv::drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, tvec,
                                    (float)(BOARD_LONG_MM * 1.2)); // truc dai ~233mm, de nhin ro
                    cv::imwrite("/tmp/extrinsic_calib_annotated.jpg", annotated);

                    std::printf(
                        "Da tinh lai. Xem lai /tmp/extrinsic_calib_annotated.jpg. "
                        "flip_x=%d flip_y=%d\n", flip_x, flip_y);
                } else if (cmd == "ok") {
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);

                    cv::FileStorage fs_out("calib/extrinsic.yaml", cv::FileStorage::WRITE);
                    fs_out << "square_size_mm" << SQUARE_SIZE_MM;
                    fs_out << "pattern_size_w" << PATTERN_SIZE.width;
                    fs_out << "pattern_size_h" << PATTERN_SIZE.height;
                    fs_out << "flip_x" << flip_x;
                    fs_out << "flip_y" << flip_y;
                    fs_out << "rvec" << rvec;
                    fs_out << "tvec" << tvec;
                    fs_out << "rotation_matrix" << R;
                    fs_out.release();

                    std::printf(
                        "\nDa luu calib/extrinsic.yaml\n"
                        "tvec (vi tri camera trong he truc robot, mm) =\n");
                    std::cout << tvec << "\n";
                    std::printf(
                        "Kiem tra: |tvec| nen xap xi chieu cao camera phia tren tam ban "
                        "ban da do tay luc lap camera.\n");
                    quit_all.store(true);
                }
            } else if (!cmd.empty()) {
                std::printf("Lenh khong hop le trong che do REVIEW. Dung: fx | fy | ok | r | q\n");
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    cam.stop();
    if (input_thread.joinable()) {
        // input_thread có thể đang chặn ở getline chờ Enter — nếu người dùng
        // đã gõ "q" thì thread tự thoát; nếu quit_all bật do lệnh khác (vd
        // "ok"), thread vẫn chờ dòng kế tiếp — detach thay vì join để không
        // treo chương trình chờ Enter thêm 1 lần.
        input_thread.detach();
    }
    return 0;
}
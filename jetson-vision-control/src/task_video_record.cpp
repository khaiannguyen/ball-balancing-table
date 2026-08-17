#include "task_video_record.hpp"
#include <cstdio>

bool TaskVideoRecord::start(TaskCameraCapture& cam,
                             const std::string& video_path,
                             const std::string& timestamps_csv_path,
                             std::chrono::steady_clock::time_point t_start,
                             double fps_hint) {
    if (running_.load(std::memory_order_relaxed)) {
        std::fprintf(stderr, "TaskVideoRecord: da chay roi\n");
        return false;
    }

    cam_       = &cam;
    video_path_ = video_path;
    fps_hint_   = fps_hint;
    t_start_    = t_start;

    ts_csv_.open(timestamps_csv_path, std::ios::out | std::ios::trunc);
    if (!ts_csv_.is_open()) {
        std::fprintf(stderr, "TaskVideoRecord: khong mo duoc %s de ghi "
                     "timestamps, HUY start (khong ghi video ma khong co "
                     "timestamp thi video vo nghia de dong bo lai sau).\n",
                     timestamps_csv_path.c_str());
        return false;
    }
    ts_csv_ << "frame_index,timestamp_ms\n";
    ts_csv_.flush();

    // VideoWriter duoc mo LAY (lazy) trong run() ngay khi co frame DAU
    // TIEN, vi can biet dung kich thuoc frame that (frame.cols/frame.rows)
    // tu GStreamer pipeline thay vi gia dinh truoc - tranh sai lech neu
    // camera_pipeline.cpp doi kich thuoc that su khac tham so yeu cau.

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::make_unique<std::thread>(&TaskVideoRecord::run, this);
    std::printf("TaskVideoRecord: da start - video: %s, timestamps: %s\n",
                video_path.c_str(), timestamps_csv_path.c_str());
    return true;
}

void TaskVideoRecord::stop() {
    if (!running_.load(std::memory_order_relaxed)) return;
    running_.store(false, std::memory_order_relaxed);
    if (thread_ && thread_->joinable()) thread_->join();

    if (writer_.isOpened()) writer_.release();
    if (ts_csv_.is_open()) ts_csv_.close();

    std::printf("TaskVideoRecord: da dung.\n");
}

void TaskVideoRecord::run() {
    cv::Mat frame;
    std::chrono::steady_clock::time_point ts;
    std::chrono::steady_clock::time_point last_ts{};
    bool has_last_ts = false;

    int64_t frame_index = 0;
    bool writer_failed_once = false; // THEM: chi in loi VideoWriter 1 lan,
                                      // tranh spam console neu no fail lien tuc

    while (running_.load(std::memory_order_relaxed)) {
        // KHONG goi task_alive_mark() o day - TaskWatchdog (task_watchdog.hpp)
        // hien chua co bit rieng cho task nay. Neu muon TaskWatchdog giam
        // sat ca task ghi video (phat hien treo/crash), them 1
        // ALIVE_BIT_VIDEO_RECORD moi vao task_watchdog.hpp/.cpp roi goi
        // task_alive_mark(ALIVE_BIT_VIDEO_RECORD) o day - TODO, chua lam
        // vi chua thay file task_watchdog that de sua an toan.

        if (!cam_->get_latest_frame(frame, ts)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        // Bo qua neu chua co frame MOI HON lan xu ly truoc - giong het
        // logic TaskBallDetect dang dung, tranh ghi trung 1 frame nhieu lan
        // khi vong lap nay chay nhanh hon camera.
        if (has_last_ts && ts == last_ts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        if (!writer_.isOpened() && !writer_failed_once) {
            int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
            writer_.open(video_path_, fourcc, fps_hint_, frame.size(), /*isColor=*/true);
            if (!writer_.isOpened()) {
                std::fprintf(stderr, "TaskVideoRecord: KHONG mo duoc "
                             "VideoWriter cho %s (kiem tra codec mp4v co "
                             "san khong, hoac doi duoi .avi). Van tiep tuc "
                             "ghi timestamps.csv nhung SE KHONG co video.\n",
                             video_path_.c_str());
                writer_failed_once = true; // SUA: chi thu open() DUNG 1 LAN,
                    // tranh goi lai moi frame neu that bai (ton CPU vo ich).
            }
        }

        if (writer_.isOpened()) {
            writer_.write(frame); // frame la BGR (dung cv::VideoWriter mac dinh)
        }

        double t_ms = std::chrono::duration<double, std::milli>(ts - t_start_).count();
        ts_csv_ << frame_index << "," << t_ms << "\n";
        ts_csv_.flush(); // moi frame - tranh mat du lieu neu Ctrl+C dot ngot
                          // (giong cach data.csv dang lam trong main.cpp)

        frame_index++;
        last_ts = ts;
        has_last_ts = true;
    }
}
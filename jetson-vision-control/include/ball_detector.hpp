#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdio>

/* BallDetector — bàn tròn màu đen, bóng màu trắng => threshold độ sáng đơn
 * giản là đủ, chưa cần Hough Circle (đúng kế hoạch J6 gốc: "dễ threshold
 * bằng ngưỡng độ sáng đơn giản trước").
 *
 * Toán học pixel -> mm giống hệt tools/sanity_check_tool.cpp (đã validate
 * khớp thực tế ở J6 bước 3.5): undistort pixel -> tia hướng trong hệ
 * camera -> chuyển sang hệ world bằng R,t từ solvePnP -> giao điểm tia với
 * mặt phẳng Z=0 (mặt bàn) -> (X,Y,0) mm trong hệ trục robot (gốc O).
 */
struct BallDetectorConfig {
    // Nguong do sang (0-255) de coi la "trang" (bong).
    // Mac dinh 75 phu hop khi camera toi/thieu sang (pixel bong ~90-100).
    // Tang len ~150-180 neu anh sang binh thuong de tranh nhan nham nen sang.
    // Giam xuong ~60 neu anh rat toi (gray mean < 50).
    // Tip: bat debug_print=true va xem gia tri gray cua vung bong thuc te.
    // Nguong do sang (0-255) ap dung SAU khi da qua CLAHE (neu bat use_clahe=true).
    // Mac dinh 120 phu hop khi anh toi, co den LED nho tren khung may.
    // - Giam xuong ~80-100 neu bong van khong detect duoc (bong qua mo/toi).
    // - Tang len ~150-180 neu nhieu nhieu nguon sang gia.
    double white_threshold = 150.0;

    // Bat CLAHE (Contrast Limited Adaptive Histogram Equalization) truoc khi
    // threshold. CLAHE tang tuong phan CUC BO -> bong mo/toi van sang ro hon
    // so voi nen, trong khi vung da sang (LED) khong bi bao hoa them.
    // Ket qua: bong duoc detect on dinh hon khi anh thieu sang khong deu.
    bool use_clahe = true;
    double clahe_clip_limit   = 2.0;  // tang neu bong van qua mo; giam neu nhieu nhieu
    int    clahe_tile_size    = 8;    // kich thuoc o CLAHE (pixel), thuong de 8

    // Loc nhieu theo dien tich pixel cua vung trang tim duoc.
    // Luu y: area tinh tren convex hull, khong phai contour thô.
    double min_area_px = 500.0;
    double max_area_px = 80000.0;

    // Do tron toi thieu = hull_area / dien_tich_hinh_tron_ly_tuong.
    // 0.75 loai tot cac vung khong tron (day vit, goc canh, day dien)
    // trong khi bong that co circularity ~0.87-0.95.
    double min_circularity = 0.75;

    // Ban kinh toi thieu/toi da (px) — TIEU CHI LOC CHINH phan biet bong
    // vs den LED nho (r~15px) va vat the lon.
    // Bong thuc te: r~43px o khoang cach nay.
    // min=25: loai sach LED (r~15-20px), giu bong (r>30px).
    // Chinh lai neu doi khoang cach camera-ban hoac kich thuoc bong.
    double min_radius_px = 38.0;
    double max_radius_px = 150.0;

    // Bat de in ra stderr dien tich + do tron cua MOI vat tron tim duoc
    // (kha ca vat bi loai vi ngoai khoang min/max_area_px) — dung de do
    // chinh xac min_area_px/max_area_px thuc te thay vi doan mo.
    bool debug_print = false;

    // ROI hinh tron gioi han vung tim kiem trong khung hinh, loai bo nen
    // ngoai ban (tuong/vat sang khac ngoai vien ban tron) khoi threshold.
    // roi_radius_px = 0 nghia la TAT mask (tim toan khung hinh).
    cv::Point2f roi_center_px{640.0f, 360.0f};
    float       roi_radius_px = 0.0f;

    // === Background subtraction (fix vat tinh sang lap lai — oc vit,
    // phan chieu tren hop servo/day dien — thinh thoang du sang de qua
    // threshold+circularity nhung KHONG PHAI bong that vi luon dung yen
    // dung 1 cho, khac han bong that di chuyen/xuat hien-bien mat) ===
    // Bat: chi chap nhan 1 vung la bong neu no SANG HON RO RET so voi nen
    // da hoc duoc (khong chi sang hon threshold co dinh). Vat tinh (oc
    // vit...) se dan "hoa vao nen" sau vai chuc frame va bi loai, trong
    // khi bong that (di chuyen/moi xuat hien) luon khac nen ro.
    bool   use_background_subtraction = true;
    // He so hoc nen moi frame (EMA). Nho -> nen hoc cham, on dinh hon,
    // nhung cham thich nghi voi thay doi anh sang that (may bat/tat den).
    double bg_learning_rate = 0.01;
    // Chenh lech do sang toi thieu (grayscale, SAU CLAHE) giua vung ung
    // vien va nen da hoc, de duoc coi la vat THAT SU khac nen (bong).
    // Tang len neu van con lot vat tinh; giam neu bong that bi loai oan.
    double bg_min_diff = 12.0;
    // So frame dau tien de nen "on dinh" truoc khi bat dau loc theo bg
    // (tranh nen chua hoc kip gay bao dong gia luc moi khoi dong).
    int    bg_warmup_frames = 60;   // ~1s o 60fps
};

class BallDetector {
public:
    BallDetector(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                 const cv::Mat& rvec, const cv::Mat& tvec,
                 BallDetectorConfig cfg = {})
        : camera_matrix_(camera_matrix), dist_coeffs_(dist_coeffs),
          rvec_(rvec), tvec_(tvec), cfg_(cfg) {
        cv::Rodrigues(rvec_, R_);
        Rinv_ = R_.t();
        cam_pos_world_ = -Rinv_ * tvec_;
        cam_z_ = cam_pos_world_.at<double>(2, 0);
    }

    // Trả true nếu tìm thấy bóng. Ghi tọa độ mm ra x_mm/y_mm. out_pixel và
    // out_radius_px (tùy chọn) để debug/overlay.
    // Khong con "const" thuc su ve mat logic (co cap nhat background_ ben
    // trong) nhung van giu chu ky const o interface — dung "mutable" cho
    // cac bien trang thai background de khong phai sua chu ky goi ham o
    // moi noi dang dung BallDetector (TaskBallDetect::run() dang goi qua
    // con tro detector_ khong doi).
    bool detect(const cv::Mat& frame_bgr, double& x_mm, double& y_mm,
                cv::Point2f* out_pixel = nullptr,
                float* out_radius_px = nullptr) const {
        cv::Mat gray;
        cv::cvtColor(frame_bgr, gray, cv::COLOR_BGR2GRAY);

        if (cfg_.roi_radius_px > 0.0f) {
            cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
            cv::circle(mask, cfg_.roi_center_px, (int)cfg_.roi_radius_px,
                       cv::Scalar(255), -1);
            cv::Mat masked;
            gray.copyTo(masked, mask);
            gray = masked;
        }

        // CLAHE tang tuong phan cuc bo truoc khi threshold — giup bong mo/toi
        // noi bat ro hon so voi nen ma khong lam bao hoa den LED sang.
        if (cfg_.use_clahe) {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
                cfg_.clahe_clip_limit,
                cv::Size(cfg_.clahe_tile_size, cfg_.clahe_tile_size));
            clahe->apply(gray, gray);
        }


        if (cfg_.debug_print) {
            std::fprintf(stderr, "gray tai bong (630,645) = %d (nguong=%.0f)\n",
                         (int)gray.at<uchar>(645, 630), cfg_.white_threshold);
            // Quet nhieu hang y quanh bong (600-690) de bat dung vi tri
            // duong ray/vien mong dang dinh voi bong (chi day vai px nen
            // scan thua truoc co the "truot" qua no).
            for (int y = 600; y <= 690; y += 5) {
                std::fprintf(stderr, "y=%d:", y);
                for (int x = 650; x <= 900; x += 15) {
                    std::fprintf(stderr, " %d", (int)gray.at<uchar>(y, x));
                }
                std::fprintf(stderr, "\n");
            }
        }

        cv::Mat binary;
        cv::threshold(gray, binary, cfg_.white_threshold, 255, cv::THRESH_BINARY);

        // Loại nhiễu nhỏ + lấp lỗ nhỏ trong vùng bóng.
        // Kernel 3x3 thay vi 5x5: kernel lon qua se lam mat bong vao nen
        // khi threshold thap (anh toi), gay mat phan tach giua bong va vat xung quanh.
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
        cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        if (cfg_.debug_print) {
            cv::imwrite("/tmp/debug_binary.png", binary);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // === BUOC 1: tim cac vung "shape_plausible" (dat area/radius/circularity)
        // TRUOC khi dung den background_ o buoc nao ca. Can lam truoc vi neu
        // day la frame DAU TIEN he thong chay (background_ con rong), ta se
        // dung ket qua nay de LOAI TRU vung bong ra khoi viec "bake" nen ban
        // dau (xem giai thich chi tiet o BUOC 2 ben duoi). ===
        double best_area = -1.0;
        int best_idx = -1;
        std::vector<int> shape_plausible_idx; // cac vung dat shape-filter, du co qua bg-check hay khong

        for (size_t i = 0; i < contours.size(); ++i) {
            std::vector<cv::Point> hull;
            cv::convexHull(contours[i], hull);
            double area = cv::contourArea(hull);

            cv::Point2f c; float r;
            cv::minEnclosingCircle(contours[i], c, r);
            double ideal_area = CV_PI * r * r;
            if (ideal_area <= 0.0) continue;
            double circularity = area / ideal_area;

            if (cfg_.debug_print && circularity > 0.5) {
                std::fprintf(stderr,
                              "[ball_detector debug] vat tron tai (%.0f,%.0f) "
                              "hull_area=%.0f circularity=%.2f radius=%.1fpx\n",
                              c.x, c.y, area, circularity, r);
            }

            if (area < cfg_.min_area_px || area > cfg_.max_area_px) continue;
            if (r < cfg_.min_radius_px || r > cfg_.max_radius_px) continue;
            if (circularity < cfg_.min_circularity) continue;

            shape_plausible_idx.push_back((int)i);
        }

        // === BUOC 2: khoi tao/cap nhat background model ===
        // SUA (fix loi "bong dung yen NGAY TU FRAME DAU TIEN bi bake vao nen
        // khoi tao"): truoc day, khi background_ con rong, code copy NGUYEN
        // frame hien tai (gray.convertTo(background_, ...)) lam nen ban dau —
        // neu bong da nam san o do tu frame 1 (dung kich ban test anh tinh),
        // nen ban dau se CHUA LUON hinh bong. Vi sau nay vung bong luon bi
        // loai khoi update_mask (khong bao gio duoc cap nhat lai), nen tai
        // vi tri bong bi "dong bang" vinh vien o gia tri sang ban dau (co
        // bong) -> mean_diff ~0 mai mai -> bong khong bao gio qua duoc bg-check.
        //
        // Fix: khi khoi tao nen lan dau, INPAINT (va) vung shape_plausible
        // bang noi suy tu pixel nen xung quanh, thay vi copy nguyen si. Nho
        // vay nen ban dau KHONG chua bong, bat ke bong co mat tu frame 1 hay
        // khong — mean_diff se cao ngay tu dau, bong duoc detect binh thuong.
        cv::Mat diff;
        bool diff_ready = false;
        if (cfg_.use_background_subtraction) {
            if (background_.empty() || background_.size() != gray.size()) {
                if (!shape_plausible_idx.empty()) {
                    cv::Mat inpaint_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
                    for (int idx : shape_plausible_idx) {
                        std::vector<cv::Point> hull;
                        cv::convexHull(contours[idx], hull);
                        std::vector<std::vector<cv::Point>> hull_wrap{hull};
                        // Ve rong hon hull mot chut (dilate qua kernel) de
                        // inpaint khong dinh lai vien bong do rin-effect.
                        cv::fillPoly(inpaint_mask, hull_wrap, cv::Scalar(255));
                    }
                    cv::dilate(inpaint_mask, inpaint_mask,
                               cv::getStructuringElement(cv::MORPH_ELLIPSE, {9, 9}));
                    cv::Mat inpainted;
                    cv::inpaint(gray, inpaint_mask, inpainted, 7, cv::INPAINT_TELEA);
                    inpainted.convertTo(background_, CV_32F);
                } else {
                    gray.convertTo(background_, CV_32F);
                }
                bg_frame_count_ = 0;
            }
            if (bg_frame_count_ >= cfg_.bg_warmup_frames) {
                cv::Mat bg_8u;
                background_.convertTo(bg_8u, CV_8U);
                cv::absdiff(gray, bg_8u, diff);
                diff_ready = true;
            }
        }

        // === BUOC 3: ap dung bg-check len tung vung shape_plausible, chon
        // best_idx (vung tron nhat, dat ca shape-filter LAN bg-check). ===
        // Dung CONVEX HULL thay vi contour tho de tinh area+circularity —
        // vet loa sang tren mat ban bong (glossy den) co the khoet khuyet 1
        // phan vien bong that, lam contour tho bi meo. Convex hull "lap"
        // phan khuyet do loa, trong khi vat that su meo/khong tron van lo ro.
        for (int idx : shape_plausible_idx) {
            std::vector<cv::Point> hull;
            cv::convexHull(contours[idx], hull);
            double area = cv::contourArea(hull);

            if (diff_ready) {
                cv::Mat hull_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
                std::vector<std::vector<cv::Point>> hull_wrap{hull};
                cv::fillPoly(hull_mask, hull_wrap, cv::Scalar(255));
                double mean_diff = cv::mean(diff, hull_mask)[0];

                if (cfg_.debug_print) {
                    std::fprintf(stderr,
                                  "[ball_detector debug]   idx=%d ...mean_diff_vs_bg=%.1f "
                                  "(can >= %.1f)\n", idx, mean_diff, cfg_.bg_min_diff);
                }

                if (mean_diff < cfg_.bg_min_diff) continue;
            }

            if (area > best_area) { best_area = area; best_idx = idx; }
        }

        // Cap nhat nen: BO QUA TAT CA vung shape_plausible (khong chi rieng
        // best_idx) de khong "hoc" bong vao nen ke ca khi bong dang tam thoi
        // bi bg-loai — chi hoc phan con lai cua khung hinh (nen that su).
        // Lam TRUOC khi return de luon chay moi frame du co detect duoc hay
        // khong.
        if (cfg_.use_background_subtraction) {
            cv::Mat update_mask = cv::Mat(gray.size(), CV_8UC1, cv::Scalar(255));
            for (int idx : shape_plausible_idx) {
                std::vector<cv::Point> hull;
                cv::convexHull(contours[idx], hull);
                std::vector<std::vector<cv::Point>> hull_wrap{hull};
                cv::fillPoly(update_mask, hull_wrap, cv::Scalar(0));
            }
            cv::Mat gray_f;
            gray.convertTo(gray_f, CV_32F);
            cv::accumulateWeighted(gray_f, background_, cfg_.bg_learning_rate, update_mask);
            if (bg_frame_count_ < cfg_.bg_warmup_frames) ++bg_frame_count_;
        }

        if (best_idx < 0) return false;

        cv::Moments m = cv::moments(contours[best_idx]);
        if (m.m00 <= 0.0) return false;
        cv::Point2f centroid((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));

        if (!pixel_to_world_mm(centroid, x_mm, y_mm)) return false;

        if (out_pixel) *out_pixel = centroid;
        if (out_radius_px) {
            float r = std::sqrt((float)(best_area / CV_PI));
            *out_radius_px = r;
        }
        return true;
    }

private:
    bool pixel_to_world_mm(const cv::Point2f& px, double& x_mm, double& y_mm) const {
        std::vector<cv::Point2d> pixel_pts{{(double)px.x, (double)px.y}};
        std::vector<cv::Point2d> norm_pts;
        cv::undistortPoints(pixel_pts, norm_pts, camera_matrix_, dist_coeffs_);
        cv::Mat ray_cam = (cv::Mat_<double>(3, 1) << norm_pts[0].x, norm_pts[0].y, 1.0);

        cv::Mat ray_world = Rinv_ * ray_cam;
        double ray_z = ray_world.at<double>(2, 0);
        if (std::abs(ray_z) < 1e-9) return false;

        double s = -cam_z_ / ray_z;
        x_mm = cam_pos_world_.at<double>(0, 0) + s * ray_world.at<double>(0, 0);
        y_mm = cam_pos_world_.at<double>(1, 0) + s * ray_world.at<double>(1, 0);
        return true;
    }

    cv::Mat camera_matrix_, dist_coeffs_, rvec_, tvec_;
    cv::Mat R_, Rinv_, cam_pos_world_;
    double  cam_z_ = 0.0;
    BallDetectorConfig cfg_;

    // === State cho background subtraction — mutable vi detect() giu
    // interface const (khong doi cach TaskBallDetect goi) nhung van can
    // cap nhat "bo nho nen" qua moi lan goi. ===
    mutable cv::Mat background_;       // CV_32F, cung kich thuoc gray sau ROI+CLAHE
    mutable int      bg_frame_count_ = 0;
};
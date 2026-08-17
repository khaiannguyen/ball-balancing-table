#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <array>
#include <cstddef>
#include <cmath>

/* ===========================================================================
 * BalanceTrajectoryController
 *
 * *** QUY DAO MOI: LUC GIAC (theo yeu cau, xem hinh tham chieu) ***
 *
 * Ten trong CODE (A, B, C ...) KHONG con trung voi ten dinh trong HINH nua.
 * Chuyen sang dat ten theo dung nhan trong hinh (A..F quanh luc giac, O la
 * tam) de khoi nham lan. task_control_loop chi quan tam kieu Waypoint{x,y}
 * tra ve tu update(), khong quan tam ten bien noi bo nen KHONG CAN sua gi.
 *
 * Van dung dung cach lam CU: setpoint NHAY THANG toi tung diem va DUNG YEN
 * trong 1 khoang thoi gian, PID luon dieu khien toi 1 DIEM CO DINH. Giua 2
 * diem CHINH lien tiep, CHEN THEM 1 diem trung diem (transit) dung RAT NGAN
 * (kTransitSeconds) de keo quy dao thuc te bam sat duong thang noi 2 diem
 * hon (ky thuat da ap dung o ban truoc, gio ap dung cho MOI canh trong
 * chuoi duong di moi).
 *
 * CHUOI DIEM CHINH (lap vo han), dung dung thu tu yeu cau:
 *   O -> C -> A -> E -> C  (tam giac 1: C-A-E)
 *     -> F -> B -> D -> F  (tam giac 2: C-F-B-D-F, noi tiep tu C truoc do)
 *     -> A -> B -> C -> D -> E -> F  (luc giac deu quanh vien)
 *     -> [DI VONG TRON quanh O]  (xem PHA VONG TRON ben duoi)
 *     -> [DI HINH SO-8 / VO CUC quanh O]  (thay the hoan toan pha BOUNCE cu
 *        da bi bo - xem SubPhase::FIGURE8 trong update())
 *     -> O (quay ve tam, roi lap lai tu dau: O -> C -> ...)
 *
 * *** DA MO LAI FULL CHUOI (het TEST MODE) ***: macro TRAJ_TEST_CIRCLE_ONLY
 * (o phan private, ngay truoc kSequence) da doi ve 0, nen chuoi THUC TE
 * chay dung nhu mo ta o tren: O -> C -> A -> E -> C -> F -> B -> D -> F ->
 * A -> B -> C -> D -> E -> F -> [vong tron] -> O -> lap lai. Thong so
 * pha CIRCLE (ban kinh/so vong/toc do) da duoc nguoi dung tinh chinh va
 * xac nhan chay tot (kCircleRadius=120mm, kCircleLaps=3, kCircleSeconds=
 * 12s) - giu nguyen, khong doi lai. Neu can quay lai TEST MODE (chi test
 * rieng pha CIRCLE, bo qua tam giac/luc giac) thi doi macro nay ve 1,
 * khong can sua gi khac.
 *
 * PHA VONG TRON (sau khi di het luc giac, truoc khi ve O):
 * *** DA SUA LAI LAN 2 (v3): tu co che NHAY-VA-DUNG-YEN (v2) QUAY LAI
 * OPEN-LOOP CHAY LIEN TUC (theo yeu cau - khong dung yen tung diem, tranh
 * cam giac "giat" khi setpoint nhay tung buoc roi rac). ***
 *
 * Lich su ngan gon:
 *   v1 (ban dau): open-loop lien tuc, toc do goc kha nhanh -> PID tre
 *       pha nhieu -> quy dao meo (elip/xoan oc), khong tron.
 *   v2: doi sang dung yen tung diem roi rac (giong TRANSIT) -> tron hon
 *       nhung MOI LAN nhay sang diem ke tiep la 1 buoc nhay setpoint dot
 *       ngot -> nguoi dung phan anh van thay "giat" ro rang tai tung
 *       buoc.
 *   v3 (HIEN TAI): quay lai open-loop lien tuc nhu v1, NHUNG lam CHAM
 *       TOC DO GOC nhieu (kCircleSeconds/kCircleLaps) dua tren so lieu
 *       thuc do duoc o v2 (dwell 1.0s/buoc 30 do bam tot -> quy doi ra
 *       toc do goc trung binh tuong duong). Vi setpoint truot lien tuc
 *       KHONG CO buoc nhay nao ca (dao ham lien tuc theo thoi gian) nen
 *       se KHONG con "giat" kieu buoc nhay nhu v2, ma se la mot duong
 *       cong LIEN TUC - do "tron" phu thuoc HOAN TOAN vao toc do goc co
 *       du cham de PID theo kip hay khong (xem kCircleSeconds, day la
 *       thong so QUAN TRONG NHAT can chinh tiep neu con giat/meo).
 *
 * Van giu: ban kinh da giam con 80mm (an toan hon so voi mep 125mm), va
 * bo diem F truoc khi vao vong tron (tranh cu nhay lon 125mm -> 80mm gay
 * dao dong keo dai nhieu giay nhu quan sat duoc trong log).
 *
 * ĐIEU KIEN QUAN TRONG - TOA DO 6 DINH LUC GIAC:
 * Hien dang GIA DINH luc giac DEU, tam O(0,0), ban kinh ngoai tiep
 * kHexRadius = 125mm, dat dung theo vi tri trong hinh (A tren-trai, B
 * tren-phai, C phai, D duoi-phai, E duoi-trai, F trai). NEU luc giac thuc
 * te tren ban khong deu / ban kinh khac / dat lech, chi can sua LAI 6
 * dong toa do trong kHexPoints[] (SAU O) - phan con lai cua code KHONG can
 * doi gi.
 * =========================================================================== */
class BalanceTrajectoryController {
public:
    // height_mm mac dinh 0 - hien tai KHONG con pha nao dieu khien height
    // nua (da BO HOAN TOAN pha BOUNCE), nen field nay luon o gia tri mac
    // dinh 0.f, giu lai trong struct de khong pha vo kieu du lieu ma
    // task_control_loop dang dung.
    struct Waypoint { float x_mm = 0.f; float y_mm = 0.f; float height_mm = 0.f; };

    BalanceTrajectoryController() = default;

    // Goi khi MOI chuyen vao Mode Balance (tu mode khac) hoac khi bong moi
    // mat roi xuat hien lai - ve lai DIEM DAU (O) cua chuoi, an toan/du
    // doan duoc hanh vi moi lan bat dau lai.
    void reset() {
        seq_index_ = 0;
        sub_phase_ = SubPhase::AT_POINT;
        phase_elapsed_s_ = 0.f;
        transit_step_ = 0;
        circle_angle_traveled_ = 0.f;
        figure8_angle_traveled_ = 0.f;
        figure8_loop_angle_ = 0.f;
        figure8_loop_index_ = 0;
        figure8_horizontal_ = true; // 2 loop dau: so 8 ngang
    }

    // Goi moi chu ky dieu khien (dt giay). Tra ve setpoint CO DINH hien tai
    // (KHONG noi suy lien tuc) theo vi tri dang chay trong chuoi, TU DONG
    // chuyen sang diem tiep theo khi het thoi gian dung (dwell/transit).
    Waypoint update(float dt) {
        phase_elapsed_s_ += dt;

        const std::size_t next_index = (seq_index_ + 1) % kNumPoints;

        switch (sub_phase_) {
        case SubPhase::AT_POINT: {
            const float dwell = (seq_index_ == 0 && post_figure8_hold_)
                                    ? kPostFigure8HoldSeconds
                                    : dwellSecondsFor(seq_index_);
            if (phase_elapsed_s_ >= dwell) {
                if (seq_index_ == 0) {
                    post_figure8_hold_ = false;
                }
                // Neu vua dung yen xong tai diem CUOI cua luc giac (F,
                // index cuoi cung truoc khi wrap ve O) thi di VONG TRON
                // quanh O truoc, chua ve O ngay.
                if (seq_index_ == kNumPoints - 1) {
                    sub_phase_ = SubPhase::CIRCLE;
                    circle_angle_traveled_ = 0.f;
                } else {
                    sub_phase_ = SubPhase::TRANSIT;
                }
                phase_elapsed_s_ = 0.f;
            }
            return kSequence[seq_index_];
        }

        case SubPhase::TRANSIT: {
            // SUA: tang tu 1 diem trung diem (0.5) len 3 diem trung gian
            // (0.25 / 0.5 / 0.75) giua 2 diem CHINH lien tiep, moi diem
            // dung yen kTransitSeconds truoc khi sang diem trung gian ke
            // tiep, giup quy dao thuc te bam sat duong thang hon nua.
            if (phase_elapsed_s_ >= kTransitSeconds) {
                phase_elapsed_s_ = 0.f;
                ++transit_step_;
                if (transit_step_ >= kNumTransitPoints) {
                    transit_step_ = 0;
                    seq_index_ = next_index;
                    sub_phase_ = SubPhase::AT_POINT;
                    return kSequence[seq_index_];
                }
            }
            const Waypoint& a = kSequence[seq_index_];
            const Waypoint& b = kSequence[next_index];
            // transit_step_ = 0 -> 0.25, 1 -> 0.5, 2 -> 0.75
            const float frac = static_cast<float>(transit_step_ + 1) /
                                static_cast<float>(kNumTransitPoints + 1);
            return Waypoint{a.x_mm + (b.x_mm - a.x_mm) * frac,
                             a.y_mm + (b.y_mm - a.y_mm) * frac, 0.f};
        }

        case SubPhase::CIRCLE: {
            // Chay lien tuc. Luu y: phase_elapsed_s_ da duoc cong dt o dau
            // update(), vi vay KHONG duoc cong dt lan nua.
            const float time_in_circle = phase_elapsed_s_;
            circle_angle_traveled_ += kCircleAngularSpeed * dt;

            if (circle_angle_traveled_ >= kCircleTotalAngle) {
                // Chuyen thang sang FIGURE8. Khong tra ve O trong mot frame
                // rieng va khong co dwell trung gian.
                circle_angle_traveled_ = kCircleTotalAngle;
                sub_phase_ = SubPhase::FIGURE8;
                phase_elapsed_s_ = 0.f;
                figure8_angle_traveled_ = 0.f;
                figure8_loop_angle_ = 0.f;
                figure8_loop_index_ = 0;
                figure8_horizontal_ = true;

                // Bat dau ngay tai O cua so 8 ngang.
                return Waypoint{0.f, 0.f, 0.f};
            }

            const float angle = kCircleStartAngle + circle_angle_traveled_;
            const float time_remaining = kCircleSeconds - time_in_circle;
            const float ramp_in  = clamp01(time_in_circle / kCircleRampSeconds);
            const float ramp_out = clamp01(time_remaining / kCircleRampSeconds);
            const float effective_radius = kCircleRadius * std::min(ramp_in, ramp_out);

            return Waypoint{effective_radius * std::cos(angle),
                             effective_radius * std::sin(angle), 0.f};
        }

        case SubPhase::FIGURE8: {
            // 4 HINH SO-8 LIEN TUC:
            //   loop 0,1 = NGANG
            //   loop 2,3 = DOC
            // Moi loop dung dung 2*pi, va duoc reset local_angle tai O.
            // Khong dung tong 8*pi de tranh nham so vong / doi huong.
            const float dt_now = dt;
            figure8_loop_angle_ += kFigure8AngularSpeed * dt_now;
            figure8_angle_traveled_ += kFigure8AngularSpeed * dt_now;

            while (figure8_loop_angle_ >= 2.f * kPi && figure8_loop_index_ < 4) {
                figure8_loop_angle_ -= 2.f * kPi;
                ++figure8_loop_index_;
            }

            if (figure8_loop_index_ >= 4) {
                // Da xong 2 ngang + 2 doc. Ve O va sau do O -> C.
                seq_index_ = 0;
                sub_phase_ = SubPhase::AT_POINT;
                phase_elapsed_s_ = 0.f;
                transit_step_ = 0;
                post_figure8_hold_ = true;
                figure8_angle_traveled_ = kFigure8TotalAngle;
                figure8_loop_angle_ = 0.f;
                figure8_loop_index_ = 4;
                return kSequence[0];
            }

            // 1 loop = 1 hinh so-8 hoan chinh va ket thuc tai O.
            // Tai dau moi loop, local angle = pi/2 -> O.
            const float t = kFigure8StartAngle + figure8_loop_angle_;

            // Chi ramp amplitude o dau. KHONG ramp-out theo tong 15s nua,
            // vi ramp-out o cuoi tong pha lam bong ve O som va dung lau,
            // khien nguoi dung thay chi ~1.5 vong. O moi loop tu nhien da
            // ve O, nen khong can ramp-out.
            const float ramp_in = clamp01(phase_elapsed_s_ / kFigure8RampSeconds);
            const float effective_amp = kFigure8Amplitude * ramp_in;

            const float s8 = std::sin(t);
            const float c8 = std::cos(t);

            if (figure8_loop_index_ < 2) {
                // 2 loop dau: SO 8 NGANG
                return Waypoint{effective_amp * c8,
                                 effective_amp * s8 * c8, 0.f};
            } else {
                // 2 loop sau: SO 8 DOC
                return Waypoint{effective_amp * s8 * c8,
                                 effective_amp * c8, 0.f};
            }
        }
        }

        return kSequence[0]; // fallback, khong bao gio xay ra
    }

private:
    enum class SubPhase { AT_POINT, TRANSIT, CIRCLE, FIGURE8 };

    // ---- Toa do 6 dinh luc giac (mm) - GIA DINH luc giac deu, R=125mm ----
    // Dat dung vi tri theo hinh: A tren-trai, B tren-phai, C phai,
    // D duoi-phai, E duoi-trai, F trai. SUA LAI 6 dong nay neu luc giac
    // thuc te khac (khong deu / ban kinh khac / dat lech goc).
    static constexpr float kHexRadius = 125.0f;
    static constexpr float kA_x = 120.25f,  kA_y = -62.5f;
    static constexpr float kB_x = 120.25f,  kB_y = 62.5f;
    static constexpr float kC_x = 0.0f,     kC_y = 125.0f;
    static constexpr float kD_x = -108.25f, kD_y = 62.5f;
    static constexpr float kE_x = -108.25f, kE_y = -62.5f;
    static constexpr float kF_x = 0.0f,     kF_y = -125.0f;
    static constexpr float kO_x = 0.0f,     kO_y = 0.0f;

    // ---- TAM THOI (TEST MODE): bypass chuoi tam giac/luc giac ----
    // De 1 de CHI chay O -> [vong tron] -> O (test rieng pha CIRCLE truoc
    // khi mo lai full chuoi). Doi ve 0 de khoi phuc NGUYEN VEN chuoi day
    // du O->C->A->E->C->F->B->D->F->A->B->C->D->E->F nhu cu, KHONG can
    // sua gi them o cho khac.
#define TRAJ_TEST_CIRCLE_ONLY 0

    // ---- Chuoi diem CHINH, dung thu tu yeu cau, lap vo han ----
    // O -> C -> A -> E -> C -> F -> B -> D -> F -> A -> B -> C -> D -> E -> F
    // (roi wrap ve O, lap lai tu dau)
#if TRAJ_TEST_BOUNCE_ONLY
    // TEST MODE 2 (BOUNCE ONLY): chuoi chi con DUY NHAT diem O. AT_POINT
    // tai O (index cuoi = 0 vi kNumPoints=1) se duoc sua o update() de di
    // THANG vao BOUNCE (khong qua CIRCLE nua).
    static constexpr std::size_t kNumPoints = 1;
    static constexpr std::array<Waypoint, kNumPoints> kSequence = {{
        {kO_x, kO_y, 0.f},  // 0: O (diem duy nhat)
    }};
#elif TRAJ_TEST_CIRCLE_ONLY
    // TEST MODE: *** DA BO diem F ***. Log thuc te (data.csv) cho thay
    // dwell 2s tai F (ban kinh 125mm, sat mep) NGAY TRUOC KHI nhay vao
    // vong tron (ban kinh 80mm) gay ra 1 cu "giat" rat manh: bong bi vot
    // ra toi 183mm roi dao dong/lac ve gan tam TOI 6 GIAY lien tuc moi on
    // dinh duoc vao duong tron - day chinh la nguyen nhan chay "giat" ban
    // thay. Bo han diem F, chi con LAI 1 diem O trong chuoi: logic san co
    // (AT_POINT tai index cuoi = kNumPoints-1 kich hoat CIRCLE) van hoat
    // dong dung vi O cung la index cuoi (kNumPoints=1). Nho vay bong di
    // THANG tu O vao vong tron (co san ramp-in tu ban kinh 0 tang dan -
    // xem SubPhase::CIRCLE trong update()), khong con cu nhay lon qua F
    // nua. Chu ky moi: O (dwell) -> CIRCLE (khong qua F) -> TRANSIT ngan
    // (O voi chinh no) -> O -> lap lai.
    static constexpr std::size_t kNumPoints = 1;
    static constexpr std::array<Waypoint, kNumPoints> kSequence = {{
        {kO_x, kO_y, 0.f},  // 0: O (diem duy nhat - dong thoi la "index cuoi"
                       //     nen tu dong kich hoat CIRCLE sau khi dwell)
    }};
#else
    static constexpr std::size_t kNumPoints = 15;
    static constexpr std::array<Waypoint, kNumPoints> kSequence = {{
        {kO_x, kO_y, 0.f},  // 0: O  (diem dau, tam)
        {kC_x, kC_y, 0.f},  // 1: C
        {kA_x, kA_y, 0.f},  // 2: A     } tam giac 1: C -> A -> E -> C
        {kE_x, kE_y, 0.f},  // 3: E
        {kC_x, kC_y, 0.f},  // 4: C
        {kF_x, kF_y, 0.f},  // 5: F
        {kB_x, kB_y, 0.f},  // 6: B     } tam giac 2: C -> F -> B -> D -> F
        {kD_x, kD_y, 0.f},  // 7: D
        {kF_x, kF_y, 0.f},  // 8: F
        {kA_x, kA_y, 0.f},  // 9: A
        {kB_x, kB_y, 0.f},  // 10: B
        {kC_x, kC_y, 0.f},  // 11: C    } luc giac deu: F -> A -> B -> C -> D -> E -> F
        {kD_x, kD_y, 0.f},  // 12: D
        {kE_x, kE_y, 0.f},  // 13: E
        {kF_x, kF_y, 0.f},  // 14: F    -> (wrap ve index 0 = O, lap lai tu dau)
    }};
#endif

    // Thoi gian dung yen tai diem O (rieng, giong kHoldCenterSeconds cu).
    static constexpr float kHoldCenterSeconds = 2.0f;
    // Thoi gian dung yen tai MOI diem chinh khac O (A..F). TODO: chinh so
    // nay neu muon nhanh/cham hon - 3.0s la gia tri ban dau hop ly de PID
    // co du thoi gian on dinh het overshoot truoc khi setpoint doi.
    static constexpr float kDwellSeconds = 2.0f;
    // Sau FIGURE8 da ve O san, chi dung rat ngan truoc khi O -> C.
    static constexpr float kPostFigure8HoldSeconds = 0.25f;
    // Thoi gian dung (rat ngan) tai diem trung diem giua 2 diem chinh lien
    // tiep - KHONG can PID on dinh het overshoot o day, chi can di ngang
    // qua de "keo" quy dao that bam sat duong thang hon. TODO: chinh neu
    // can - 0.5s la gia tri ban dau hop ly.
    static constexpr float kTransitSeconds = 0.2f;
    // SO DIEM TRUNG GIAN chen giua 2 diem chinh lien tiep (truoc la 1,
    // gio tang len 3: 0.25 / 0.5 / 0.75 theo duong thang noi 2 diem).
    static constexpr int kNumTransitPoints = 3;

    static float dwellSecondsFor(std::size_t index) {
        return (index == 0) ? kHoldCenterSeconds : kDwellSeconds;
    }

    // ---- Pha di VONG TRON quanh O sau khi het luc giac ----
    // DA SUA (v3): QUAY LAI open-loop chay lien tuc (KHONG dung yen tung
    // diem nua, theo yeu cau) - xem giai thich chi tiet o comment dau
    // file va trong nhanh SubPhase::CIRCLE cua update(). Toc do goc va
    // ban kinh van GIU CAC GIA TRI DA TINH CHINH tu log thuc te (v2) de
    // han che tre pha PID nhieu nhat co the trong khi van la open-loop.
    static constexpr float kPi = 3.14159265358979f;
    // Ban kinh vong tron (mm). *** DA GIAM (dua tren log thuc te data.csv):
    // voi ban kinh lenh 130mm, bong THUC TE vot ra toi 158mm (vuot mep
    // luc giac 125mm - nguy co roi bong). Giam xuong 80mm de co bien an
    // toan ngay ca khi con overshoot. TODO: neu van thay bong ra gan/qua
    // mep luc giac trong log moi, GIAM tiep so nay.
    static constexpr float kCircleRadius = 120.0f;
    // So vong muon di. TODO: chinh cho phu hop yeu cau thuc te.
    static constexpr int kCircleLaps = 2;
    // Tong thoi gian di het so vong tren (giay) -> suy ra toc do goc =
    // kCircleTotalAngle / kCircleSeconds. *** QUAN TRONG: day la thong so
    // CHINH de kiem soat do "tron" khi chay open-loop lien tuc *** - toc
    // do goc CANG CHAM thi PID cang it bi tre pha, duong di thuc te cang
    // tron. Dang de 12s/vong (tuong duong toc do trung binh ~0.52 rad/s)
    // - gia tri nay suy ra tu thoi gian dwell 1.0s/buoc-30-do da CHUNG
    // MINH bam sat tot khi con dung co che diem-dung-yen (v2). TODO: NEU
    // VAN CON GIAT/MEO HINH, day la thong so DAU TIEN can THU TANG (vd
    // 16-20s/vong) de quay cham hon nua.
    static constexpr float kCircleSeconds = 10.0f;
    static constexpr float kCircleTotalAngle = 2.0f * kPi * static_cast<float>(kCircleLaps);
    static constexpr float kCircleAngularSpeed = kCircleTotalAngle / kCircleSeconds;
    // Thoi gian tang dan / giam dan ban kinh o dau/cuoi pha CIRCLE (giay),
    // tranh giat ly tam dot ngot luc bat dau/ket thuc vong tron. TODO:
    // tang so nay len neu van con hien tuong tran ban kinh. Luu y:
    // 2*kCircleRampSeconds phai NHO HON kCircleSeconds.
    static constexpr float kCircleRampSeconds = 0.75f;
    static float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

    // ---- Hinh dang xung BOUNCE bat doi xung (day nhanh / ha cham) ----
    // Tra ve gia tri chuan hoa [0,1] theo pha [0,1) trong 1 chu ky:
    //   0            -> vi tri THAP (day)
    //   rise_fraction -> vi tri CAO NHAT (dinh) - dat toi day voi VAN TOC
    //                    LON NHAT (ease-in: cang gan dinh cang nhanh)
    //   1 (= chu ky sau) -> quay lai vi tri THAP, nhung EM, VAN TOC ~0
    // DIEM MAU CHOT: tai chinh moc rise_fraction, van toc NHAY tu "lon
    // nhat" (cuoi pha day) xuong "0" (dau pha ha) CHI TRONG 1 dt - day la
    // cho gay gia toc/giam toc LON NHAT trong ca chu ky, can thiet de VUOT
    // gia toc trong truong g (~9810mm/s^2) va lam bong tach khoi mat ban.
    // Day KHONG PHAI bug hay thieu sot lam muot - la CO Y, vi day chinh la
    // "cu hat" tao luc phong bong ma song sin (dao ham lien tuc hoan toan,
    // gia toc trai deu ca chu ky) khong the nao tao ra duoc o bien do/tan
    // so an toan cho servo.
    // Goc bat dau vong tron = goc cua diem F (0, -125mm) tinh tu tam O,
    // de setpoint bat dau vong tron ngay gan vi tri F, do do "muot" hon
    // thay vi nhay xa. F nam tren truc -Y nen goc = -90 do = -pi/2.
    static constexpr float kCircleStartAngle = -kPi * 0.5f;

    // ---- Pha di HINH SO-8 / VO CUC quanh O, sau khi het Circle ----
    // THAY THE HOAN TOAN pha BOUNCE cu (da bo). Xem giai thich chi tiet
    // cong thuc tham so hoa trong nhanh SubPhase::FIGURE8 cua update().
    //
    // Goc bat dau (t=pi/2) de duong x(t)=A*cos(t), y(t)=A*sin(t)*cos(t)
    // xuat phat dung tai O (0,0) - khop lien tuc voi diem CIRCLE ket thuc
    // ngay truoc do (cung da ramp-out ve gan O).
    static constexpr float kFigure8StartAngle = kPi * 0.5f;
    // Bien do hinh so-8 (mm) - khoang cach tu tam O den diem xa nhat cua
    // moi vong tron con (tuc "chieu rong" moi nua cua hinh so-8). Dat
    // AN TOAN duoi kHexRadius=125mm giong kCircleRadius. TODO: chinh neu
    // muon hinh so-8 to/nho hon.
    static constexpr float kFigure8Amplitude = 90.0f;
    // Tong so hinh so-8: 4 hinh, gom 2 NGANG + 2 DOC.
    // Moi hinh so-8 dung 2*pi rad.
    static constexpr int kFigure8Laps = 4;
    // Tong thoi gian di het so vong tren (giay) -> suy ra toc do goc.
    // Giong CIRCLE, CANG CHAM thi PID cang it tre pha, duong di cang
    // tron. TODO: day la thong so DAU TIEN can tang neu con giat/meo
    // hinh so-8 (vd tang len 16-20s/vong de quay cham hon).
    static constexpr float kFigure8Seconds = 24.0f;
    static constexpr float kFigure8TotalAngle = 2.0f * kPi * static_cast<float>(kFigure8Laps);
    static constexpr float kFigure8AngularSpeed = kFigure8TotalAngle / kFigure8Seconds;
    // Thoi gian tang dan / giam dan bien do o dau/cuoi pha FIGURE8 (giay),
    // tranh giat dot ngot luc bat dau/ket thuc. Luu y: 2*kFigure8RampSeconds
    // phai NHO HON kFigure8Seconds.
    static constexpr float kFigure8RampSeconds = 0.30f;

    std::size_t seq_index_ = 0;
    SubPhase sub_phase_ = SubPhase::AT_POINT;
    float phase_elapsed_s_ = 0.f;
    int transit_step_ = 0; // 0..kNumTransitPoints-1, vi tri trong TRANSIT
    bool post_figure8_hold_ = false; // O ngan truoc khi bat dau chu ky O -> C
    bool figure8_horizontal_ = true; // 2 loop dau la ngang, 2 loop sau la doc
    int figure8_loop_index_ = 0;     // 0..3: 2 ngang + 2 doc
    float figure8_loop_angle_ = 0.f; // 0..2*pi cho tung hinh so-8
    float circle_angle_traveled_ = 0.f;
    float figure8_angle_traveled_ = 0.f;
};

// Alias de code goi noi khac (task_control_loop.cpp dang dung kieu tra ve
// ten "Waypoint" o pham vi global) van bien dich duoc khong doi gi.
using Waypoint = BalanceTrajectoryController::Waypoint;

#endif
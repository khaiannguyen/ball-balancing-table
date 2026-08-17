#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>

/* PIDController — dùng cho J7 Task_Control_Loop, 1 instance/trục (X, Y).
 *
 * Chong windup gom 3 co che ket hop (ap dung trong update_with_velocity(),
 * ham dang duoc TaskControlLoop su dung thuc te):
 *   1. Conditional integration (cu): khong cong don integral neu output
 *      DA/SE saturate theo chieu lam error te hon.
 *   2. Integral clamp (MOI): gioi han cung truc tiep cho ban than
 *      integral_ (doc lap voi gioi han output) — tranh integral tich luy
 *      vo han khi error ton tai dai dang nhung CHUA du lam output
 *      saturate (vd bong dung im gan tam do ma sat/meo, khong dung yen
 *      dung tam).
 *   3. Integral leak / decay (MOI): moi chu ky, integral_ tu "ro ri" giam
 *      nhe truoc khi cong them error moi — tranh integral "nho" sai so cu
 *      qua lau.
 *   4. Zero-crossing reset (MOI, quan trong nhat cho van de overshoot):
 *      khi error DOI DAU (bong vuot qua setpoint sang phia doi dien),
 *      integral tich luy tu phia CU khong con y nghia cho phia MOI —
 *      reset integral_ ve 0 ngay luc do, tranh "quan tinh integral" tiep
 *      tuc day bong qua da sang phia ben kia, gay dao dong tang dan bien
 *      do (mat on dinh dai han) dung nhu hien tuong da quan sat thuc te. */
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max)
        : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {
        // Mac dinh: gioi han integral = bien do output toi da. Day la gia
        // tri "an toan" hop ly — integral mot minh no (khi ki_ * integral_)
        // khong the goi ra output vuot qua gioi han vat ly, ke ca khi
        // P/D dang bang 0. Co the doi lai bang set_integral_limit().
        integral_limit_ = std::fabs(out_max_);
    }

    // THEM: gioi han cung cho integral. Goi SAU constructor neu muon gia
    // tri khac mac dinh (= out_max_).
    void set_integral_limit(float limit) { integral_limit_ = std::fabs(limit); }

    // THEM: he so "ro ri" moi chu ky, trong khoang (0, 1]. 1.0 = KHONG ro
    // ri (mac dinh, giu hanh vi cu). Cang nho hon 1 thi integral cang
    // "quen" sai so cu nhanh hon. Vi du 0.999 o 100Hz nghia la sau ~1s,
    // integral con lai ~90% (0.999^100 ≈ 0.905) — ro ri rat cham, chi de
    // tranh tich luy VO HAN trong thoi gian rat dai, khong anh huong dang
    // ke toi qua trinh "thang ma sat tinh" dang can trong vai giay.
    void set_integral_leak(float leak_per_cycle) { integral_leak_ = leak_per_cycle; }

    // THEM: deadband cho error (cung don vi voi setpoint/measured, vd mm).
    // |error| < deadband -> coi nhu = 0 (khong cong P, khong cong integral,
    // khong tinh la "doi dau" cho zero-crossing reset). Muc dich: nhieu do
    // vi tri bong (+-1..2mm quan sat thuc te tren data.csv) khong duoc coi
    // la "bong da lech tam" nua, tranh servo rung theo nhieu do khi bong
    // thuc su da dung yen gan tam.
    void set_error_deadband(float deadband) { error_deadband_ = std::fabs(deadband); }

    // THEM: he so loc thong thap (EMA) cho van toc do duoc, alpha trong
    // (0,1]. alpha = 1.0 = KHONG loc (mac dinh, giu hanh vi cu). Alpha
    // cang nho thi van toc cang muot nhung tre pha cang nhieu. Muc dich:
    // van toc tinh tu sai phan vi tri camera framerate thap rat nhay voi
    // nhieu phat hien bong (+-1..2mm/frame) -> D-term "gian" theo nhieu
    // do thay vi chuyen dong that cua bong, day chinh la nguyen nhan gay
    // rung khi bong da o tam (quan sat tren data.csv: roll_d/pitch_d dao
    // dong -0.15..0.44 deg lien tuc doi dau trong khi Ballx,Bally chi
    // +-1..2mm, tuc la nhieu do chu khong phai bong di chuyen that).
    void set_velocity_filter_alpha(float alpha) { vel_filter_alpha_ = alpha; }

    float update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;

        float p_term = kp_ * error;
        float d_term = (dt > 1e-6f) ? kd_ * (error - last_error_) / dt : 0.f;

        // Thử tính output NẾU cộng dồn integral bình thường, xem có vượt
        // giới hạn theo chiều làm error tệ hơn không (conditional integration).
        float trial_output = p_term + ki_ * integral_ + d_term;
        bool saturated_high = trial_output > out_max_;
        bool saturated_low  = trial_output < out_min_;
        bool would_worsen = (saturated_high && error > 0) || (saturated_low && error < 0);

        if (!would_worsen) {
            integral_ += error * dt;
        }

        float output = p_term + ki_ * integral_ + d_term;
        if (output > out_max_) output = out_max_;
        if (output < out_min_) output = out_min_;

        last_error_ = error;
        return output;
    }

    // update_with_velocity() — dung VAN TOC DO THAT lam D-term (xem giai
    // thich chi tiet o phien ban truoc: tranh dao ham tren tin hieu vi tri
    // dang "bac thang" do tan so camera thap hon tan so PID loop). Ham nay
    // duoc TaskControlLoop goi trong thuc te, nen 3 co che chong windup
    // MOI (2,3,4 o tren) duoc tich hop VAO DAY.
    float update_with_velocity(float setpoint, float measured,
                                float measured_velocity, float dt) {
        float error = setpoint - measured;

        // THEM: deadband - error nam trong nguong nhieu do thi coi nhu = 0.
        // Ap dung TRUOC khi xet zero-crossing va truoc khi tinh P/integral,
        // de nhieu do +-1..2mm quanh tam khong bi hieu la "doi dau lien
        // tuc" va khong lam P-term giat theo nhieu.
        if (std::fabs(error) < error_deadband_) {
            error = 0.f;
        }

        // Co che 4 - zero-crossing reset: CHI reset khi error thuc su doi
        // dau RA KHOI vung deadband (khong con la 0.f do deadband o tren),
        // tuc bong thuc su vuot qua setpoint sang phia doi dien ro rang -
        // tranh reset lien tuc chi vi nhieu do dao dong quanh 0.
        bool sign_changed = (error > 0.f && last_error_ < 0.f) ||
                             (error < 0.f && last_error_ > 0.f);
        if (sign_changed) {
            integral_ = 0.f;
        }

        // Co che 3 - integral leak: ro ri nhe truoc khi cong them error
        // moi. integral_leak_ = 1.0 mac dinh nghia la KHONG ro ri.
        integral_ *= integral_leak_;

        // THEM: loc thong thap (EMA) cho van toc do duoc TRUOC khi dung
        // lam D-term - giam nhieu do vi tri camera bi khuech dai qua phep
        // sai phan/dt thanh "van toc gia" khi bong that ra dang dung yen.
        filtered_velocity_ += vel_filter_alpha_ * (measured_velocity - filtered_velocity_);

        float p_term = kp_ * error;
        float d_term = -kd_ * filtered_velocity_;

        // Co che 1 - conditional integration (anti-windup cu, GIU NGUYEN).
        float trial_output = p_term + ki_ * integral_ + d_term;
        bool saturated_high = trial_output > out_max_;
        bool saturated_low  = trial_output < out_min_;
        bool would_worsen = (saturated_high && error > 0) || (saturated_low && error < 0);

        if (!would_worsen) {
            integral_ += error * dt;
        }

        // Co che 2 - integral clamp: gioi han cung TRUC TIEP cho integral_,
        // DOC LAP voi viec output co saturate hay chua. Day la "luoi an
        // toan" thu 2: du co che 1 khong kich hoat (vi output CHUA
        // saturate), integral_ van khong the vuot qua gioi han nay - ngan
        // tich luy vo han khi error ton tai dai dang o muc nho (vd bong
        // dung gan nhung khong dung tam, dung nhu truong hop thuc te
        // ball=(-20,28)mm da quan sat).
        if (integral_ > integral_limit_)  integral_ = integral_limit_;
        if (integral_ < -integral_limit_) integral_ = -integral_limit_;

        float output = p_term + ki_ * integral_ + d_term;
        if (output > out_max_) output = out_max_;
        if (output < out_min_) output = out_min_;

        last_error_ = error;
        return output;
    }

    // Gọi khi mất bóng / mất kết nối STM32 / khởi động lại, tránh windup
    // và tránh d_term "giật" khi bóng xuất hiện lại sau 1 khoảng trống.
    void reset() {
        integral_ = 0.f;
        last_error_ = 0.f;
        filtered_velocity_ = 0.f;
    }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float integral_ = 0.f;
    float last_error_ = 0.f;
    float integral_limit_;             // gan trong constructor (= |out_max_|)
    float integral_leak_ = 1.0f;       // mac dinh: khong ro ri
    float error_deadband_ = 0.f;       // mac dinh: khong deadband (giu hanh vi cu)
    float vel_filter_alpha_ = 1.0f;    // mac dinh: khong loc (giu hanh vi cu)
    float filtered_velocity_ = 0.f;
};

#endif
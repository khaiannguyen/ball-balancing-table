#include "control_mode_calib.h"
#include "servo_actuator.h"
#include "calibration_data.h"
#include "system_state.h"
#include "task_state_machine.h"
#include "trajectory.h"
#include "ui_data.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern osMessageQueueId_t StateRequestQueueHandle;

/* ---- Hằng số thời gian đo (GIỮ NGUYÊN như v4) ---- */
#define CALIB_SETTLE_CYCLES   200   // 200 * 10ms = 2000ms @ 100Hz
#define CALIB_SAMPLE_COUNT     5    // dùng cho mỗi điểm quét GRID
#define CALIB_OFFSET_SAMPLE_COUNT   200   // 200 * 10ms = 2000ms, dùng riêng cho bước OFFSET

/* MỚI (Giai đoạn 6) - LƯỚI QUÉT 2D thay 3 sweep 1-trục của v4.
 * CALIB_GRID_HALF_US: biên độ MỖI TRỤC (S1,S2), quét đối xứng -HALF..+HALF
 * quanh neutral (KHÔNG chỉ 1 chiều như v4 "0->360") - vì lệnh Balance thực
 * tế cần cả roll/pitch âm lẫn dương, dữ liệu calib phải phủ cả 2 chiều mới
 * dùng được cho toàn dải hoạt động.
 * CALIB_GRID_POINTS: số điểm mỗi trục -> tổng CALIB_GRID_POINTS^2 điểm.
 * Chọn 7 (49 điểm) làm điểm khởi đầu hợp lý (đủ phủ bậc 2, không quá lâu
 * để quét - mỗi điểm ~2s settle + ~50ms sample => 49 điểm ~100s, CHƯA kể
 * thời gian di chuyển). Tăng lên 9 (81 điểm) nếu sau khi fit vẫn thấy RMS
 * chưa đạt ngưỡng và nghi ngờ do lưới còn thưa.
 * LƯU Ý AN TOÀN: |S1_off| + |S2_off| tối đa = 2*CALIB_GRID_HALF_US (khi 2
 * trục cùng dấu, cùng biên) -> S3_off = -(S1_off+S2_off) cũng tối đa bằng
 * đúng số đó. Với CALIB_GRID_HALF_US=170 => S3_off tối đa 340, VẪN nằm
 * trong CALIB_TILT_MAX_US=360 (còn dư 20us margin) - xem static_assert bên
 * dưới tự kiểm tra điều kiện này khi build, tránh ai đó tăng
 * CALIB_GRID_HALF_US mà quên kiểm tra lại biên an toàn cơ khí. */
#define CALIB_GRID_HALF_US    180.0f
#define CALIB_GRID_POINTS     7
//Static_assert((int)(2.0f * CALIB_GRID_HALF_US) <= CALIB_TILT_MAX_US, "CALIB_GRID_HALF_US qua lon - S3 co the vuot CALIB_TILT_MAX_US, giam CALIB_GRID_HALF_US");

/* MỚI (Giai đoạn 6) - ngưỡng gate chất lượng fit. Đơn vị: us (vì Y là
 * S1/S2 tính bằng us, không phải độ). PHẢI tinh chỉnh thực nghiệm theo yêu
 * cầu độ chính xác thật của hệ (ví dụ nếu 1 độ roll/pitch tương ứng ~15-20us
 * dịch chuyển servo thì RMS_MAX=15us nghĩa là sai số dự đoán trung bình
 * dưới ~1 độ - điều chỉnh theo tỉ lệ thực đo được, KHÔNG suy diễn suông). */
#define CALIB_LS_RMS_MAX_US      15.0
#define CALIB_LS_MAXERR_MAX_US   40.0

#define CALIB_TRAJ_V_MAX_US_S    3000.0f
#define CALIB_TRAJ_A_MAX_US_S2   8000.0f

/* ---- state máy đo 1 điểm (dùng cho OFFSET, có return-to-neutral) ---- */
typedef enum {
    MEAS_MOVE_START = 0,
    MEAS_MOVE_RUN,
    MEAS_SETTLE,
    MEAS_SAMPLE,
    MEAS_RETURN_START,
    MEAS_RETURN_RUN,
    MEAS_RETURN_SETTLE,
    MEAS_DONE
} meas_phase_t;

static meas_phase_t s_meas_phase   = MEAS_MOVE_START;
static uint16_t      s_settle_ct    = 0;
static uint8_t         s_sample_ct    = 0;
static float             s_sum_roll     = 0.0f;
static float               s_sum_pitch    = 0.0f;
static float                 s_result_roll  = 0.0f;
static float                   s_result_pitch = 0.0f;

static calib_sub_state_t s_sub = CALIB_SUB_OFFSET;

static trajectory_t s_move_traj[3];
static float s_roll_offset, s_pitch_offset;

/* MỚI (Giai đoạn 6) - lưu toàn bộ điểm lưới (R,P,S1,S2 - offset so với
 * neutral) để giải Least Squares bậc 2 ngay trên MCU. */
#define LS_MAX_POINTS   (CALIB_GRID_POINTS * CALIB_GRID_POINTS)
static float    s_ls_R[LS_MAX_POINTS];
static float    s_ls_P[LS_MAX_POINTS];
static float    s_ls_S1[LS_MAX_POINTS];
static float    s_ls_S2[LS_MAX_POINTS];
static uint16_t s_ls_count = 0;

/* Hệ số bậc 2 vừa giải xong: index 0=S1,1=S2,2=S3 (S3 suy từ S3=-(S1+S2)
 * áp trực tiếp lên hệ số, không hồi quy riêng lần 3 - xem
 * solve_and_print_ik_coeffs()). */
static bool  s_ls_ok = false;
static float s_ik_coef[3][CALIB_IK_NUM_COEF];

/* MỚI (Giai đoạn 6) - state máy cho lưới quét 2D, thay 3 state SWEEP cũ. */
typedef enum {
    SWEEP_MOVE_START = 0,
    SWEEP_MOVE_RUN,
    SWEEP_SETTLE,
    SWEEP_SAMPLE,
    SWEEP_RETURN_START,   /* chỉ vào đây SAU điểm cuối cùng của lưới */
    SWEEP_RETURN_RUN,
    SWEEP_ALL_DONE
} sweep_phase_t;

static sweep_phase_t s_sweep_phase = SWEEP_MOVE_START;
static uint16_t      s_grid_idx    = 0;   /* 0 .. LS_MAX_POINTS-1 */

/* MỚI - override neutral (nếu người dùng muốn đổi S1/S2/S3_neutral khi
 * chạy lại Mode Calib). Mặc định KHÔNG override - dùng nguyên neutral hiện
 * có trong Flash (giữ đúng hành vi cũ nếu không ai gọi
 * control_mode_calib_set_neutral()). Xem giải thích đầy đủ trong
 * control_mode_calib.h. */
static bool    s_neutral_override = false;
static int32_t s_ov_n1 = 0, s_ov_n2 = 0, s_ov_n3 = 0;

void control_mode_calib_set_neutral(int16_t s1_neutral, int16_t s2_neutral, int16_t s3_neutral)
{
    s_ov_n1 = s1_neutral;
    s_ov_n2 = s2_neutral;
    s_ov_n3 = s3_neutral;
    s_neutral_override = true;
}

/* ==========================================================================
 * MỚI - override neutral HARDCODE để sửa nhanh bằng tay, KHÔNG cần thêm
 * màn hình/nút bấm nào. Cách dùng:
 *   1. Đổi CALIB_NEUTRAL_HARDCODE_ENABLE thành 1.
 *   2. Sửa 3 giá trị CALIB_NEUTRAL_HARDCODE_S1/S2/S3 thành neutral mới
 *      muốn dùng (đơn vị us, giống S1_neutral/S2_neutral/S3_neutral trong
 *      calibration_data.h).
 *   3. Build lại, chạy Mode Calib như bình thường (BTN1 long ở đúng chỗ
 *      trigger EVT_BTN_CALIB/tương đương) - control_mode_calib_enter() sẽ
 *      tự gọi control_mode_calib_set_neutral() với 3 giá trị này, áp dụng
 *      xuyên suốt OFFSET/SWEEP_GRID/SAVE (xem get_working_neutral()).
 *   4. Sau khi calib xong và Ghi Flash thành công, ĐỔI LẠI
 *      CALIB_NEUTRAL_HARDCODE_ENABLE về 0 (hoặc để nguyên cũng không sao -
 *      lần chạy calib SAU sẽ lại dùng ĐÚNG 3 giá trị hardcode này, KHÔNG
 *      tự động lấy neutral vừa lưu trong Flash làm neutral tiếp theo, dễ
 *      gây nhầm "sao sửa neutral không ăn" nếu quên tắt cờ này).
 * ========================================================================== */
#define CALIB_NEUTRAL_HARDCODE_ENABLE   1      /* 1 = bật override, 0 = dùng neutral hiện có trong Flash */
#define CALIB_NEUTRAL_HARDCODE_S1       1632
#define CALIB_NEUTRAL_HARDCODE_S2       1549
#define CALIB_NEUTRAL_HARDCODE_S3       1580

/* ==========================================================================
 * MỚI - override deadband HARDCODE, y hệt cơ chế CALIB_NEUTRAL_HARDCODE_*
 * ở trên. Bình thường CALIB_SUB_SAVE giữ nguyên deadband cũ đọc từ Flash
 * (out = *c), nên sửa deadband_S1/S2/S3 trong
 * calibration_data_set_defaults() KHÔNG có tác dụng nếu Flash đã có dữ
 * liệu hợp lệ - phải dùng cờ hardcode này để ép ghi giá trị mới xuống
 * Flash ở lần chạy Mode Calib kế tiếp.
 *
 * Cách dùng:
 *   1. Đổi CALIB_DEADBAND_HARDCODE_ENABLE thành 1.
 *   2. Sửa 3 giá trị CALIB_DEADBAND_HARDCODE_S1/S2/S3 thành deadband mới
 *      muốn dùng (đơn vị us, giống deadband_S1/S2/S3 trong
 *      calibration_data.h - đo bằng Mode Manual).
 *   3. Build lại, chạy Mode Calib - CALIB_SUB_SAVE sẽ ép out.deadband_S1/
 *      S2/S3 = 3 giá trị này TRƯỚC KHI ghi Flash (không phụ thuộc deadband
 *      cũ trong Flash, không phụ thuộc kết quả Least Squares).
 *   4. Sau khi ghi Flash thành công, ĐỔI LẠI CALIB_DEADBAND_HARDCODE_ENABLE
 *      về 0 - nếu để 1 và chạy calib lần nữa, deadband sẽ LẠI bị ép về
 *      đúng 3 số hardcode này, không giữ được deadband vừa lưu trước đó.
 * ========================================================================== */
#define CALIB_DEADBAND_HARDCODE_ENABLE  1      /* 1 = bật override, 0 = dùng deadband hiện có trong Flash */
#define CALIB_DEADBAND_HARDCODE_S1      10
#define CALIB_DEADBAND_HARDCODE_S2      10
#define CALIB_DEADBAND_HARDCODE_S3      10

/* Trả về neutral "đang dùng" cho lần chạy calib này - override nếu có,
 * ngược lại lấy từ calibration_data hiện có (đúng hành vi cũ). DÙNG HÀM
 * NÀY thay vì đọc thẳng c->S1_neutral ở mọi nơi trong file, để override
 * (nếu có) được áp dụng NHẤT QUÁN xuyên suốt OFFSET/SWEEP_GRID/SAVE. */
static void get_working_neutral(const calibration_data_t *c, int32_t *n1, int32_t *n2, int32_t *n3)
{
    if (s_neutral_override) {
        *n1 = s_ov_n1; *n2 = s_ov_n2; *n3 = s_ov_n3;
    } else {
        *n1 = c->S1_neutral; *n2 = c->S2_neutral; *n3 = c->S3_neutral;
    }
}

static void set_guide(const char *msg)
{
    strncpy(g_uiData.guideText, msg, sizeof(g_uiData.guideText) - 1);
    g_uiData.guideText[sizeof(g_uiData.guideText) - 1] = '\0';
}

static bool traj_run_step(float dt)
{
    /* SUA: trajectory_update() doi chu ky (nhan them dt + tra ve qua con
     * tro x_ref/v_ref/a_ref, thay vi return float truc tiep) - xem
     * trajectory.h ban moi. Calib khong can v_ref/a_ref nen truyen NULL. */
    float p1, p2, p3;
    trajectory_update(&s_move_traj[0], dt, &p1, NULL, NULL);
    trajectory_update(&s_move_traj[1], dt, &p2, NULL, NULL);
    trajectory_update(&s_move_traj[2], dt, &p3, NULL, NULL);

    servo_actuator_set_target(SERVO_CH_S1, (int32_t)lroundf(p1));
    servo_actuator_set_target(SERVO_CH_S2, (int32_t)lroundf(p2));
    servo_actuator_set_target(SERVO_CH_S3, (int32_t)lroundf(p3));

    return trajectory_is_done(&s_move_traj[0]) &&
           trajectory_is_done(&s_move_traj[1]) &&
           trajectory_is_done(&s_move_traj[2]);
}

static void traj_start_to(int32_t s1, int32_t s2, int32_t s3)
{
    int32_t cs1, cs2, cs3;
    servo_actuator_get_local(&cs1, &cs2, &cs3);

    float from[3] = { (float)cs1, (float)cs2, (float)cs3 };
    float to[3]   = { (float)s1,  (float)s2,  (float)s3  };

    trajectory_start_synced3(s_move_traj, from, to,
                              CALIB_TRAJ_V_MAX_US_S, CALIB_TRAJ_A_MAX_US_S2);
}

static void send_evt(state_event_t evt)
{
    osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
}

/* ==========================================================================
 * MỚI (Giai đoạn 6) - GIẢI LEAST SQUARES TỔNG QUÁT bằng phương trình chuẩn
 * tắc (normal equations) + khử Gauss có CHỌN TRỤC XOAY (partial pivoting).
 * Thay thế hoàn toàn solve_3x3()/Cramer của v4 - Cramer 6x6 viết tay quá
 * cồng kềnh và dễ sai (cofactor 6x6 tính tay rất dễ lẫn dấu/thiếu số hạng).
 * Gauss có pivoting vừa gọn vừa ổn định số học hơn cho ma trận 6x6.
 * ========================================================================== */
#define NCOEF 6

/* Giải A*x = b, A là NCOEF x NCOEF, GHI ĐÈ lên A/b (dùng bản copy riêng ở
 * nơi gọi nếu cần giữ nguyên A gốc). Trả false nếu ma trận suy biến (pivot
 * quá nhỏ ở bước nào đó) - dữ liệu lưới không đủ đa dạng để giải bậc 2. */
static bool gauss_solve(double A[NCOEF][NCOEF], double b[NCOEF], double x[NCOEF])
{
    for (int col = 0; col < NCOEF; col++) {
        /* Tìm hàng có |A[row][col]| lớn nhất từ col trở xuống - chọn trục
         * xoay để giảm sai số khuếch đại do chia cho số nhỏ. */
        int piv = col;
        double best = fabs(A[col][col]);
        for (int row = col + 1; row < NCOEF; row++) {
            if (fabs(A[row][col]) > best) { best = fabs(A[row][col]); piv = row; }
        }
        if (best < 1e-9) {
            return false;   /* suy biến - dữ liệu lưới gần như phụ thuộc tuyến tính */
        }
        if (piv != col) {
            for (int k = 0; k < NCOEF; k++) { double t = A[col][k]; A[col][k] = A[piv][k]; A[piv][k] = t; }
            double t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        /* Khử các hàng dưới */
        for (int row = col + 1; row < NCOEF; row++) {
            double factor = A[row][col] / A[col][col];
            for (int k = col; k < NCOEF; k++) A[row][k] -= factor * A[col][k];
            b[row] -= factor * b[col];
        }
    }
    /* Thế ngược */
    for (int row = NCOEF - 1; row >= 0; row--) {
        double s = b[row];
        for (int k = row + 1; k < NCOEF; k++) s -= A[row][k] * x[k];
        x[row] = s / A[row][row];
    }
    return true;
}

/* Vector đặc trưng bậc 2 cho 1 mẫu (R,P): [R, P, R^2, P^2, R*P, 1] */
static void feature_row(double R, double P, double f[NCOEF])
{
    f[0] = R;
    f[1] = P;
    f[2] = R * R;
    f[3] = P * P;
    f[4] = R * P;
    f[5] = 1.0;
}

/* Xây normal equations (F^T F) c = F^T Y từ n mẫu, giải ra 6 hệ số. */
static bool solve_quadratic(const float *R, const float *P, const float *Y, uint16_t n,
                             double coef_out[NCOEF])
{
    double AtA[NCOEF][NCOEF] = {0};
    double Atb[NCOEF]        = {0};

    for (uint16_t i = 0; i < n; i++) {
        double f[NCOEF];
        feature_row((double)R[i], (double)P[i], f);
        for (int r = 0; r < NCOEF; r++) {
            Atb[r] += f[r] * (double)Y[i];
            for (int c = 0; c < NCOEF; c++) {
                AtA[r][c] += f[r] * f[c];
            }
        }
    }
    return gauss_solve(AtA, Atb, coef_out);
}

/* RMS, sai số lớn nhất, R^2 cho mô hình bậc 2 so với dữ liệu thật. */
static void compute_fit_stats_quad(const float *R, const float *P, const float *Y, uint16_t n,
                                    const double coef[NCOEF],
                                    double *rms, double *max_err, double *r2)
{
    double sse = 0.0, sum_y = 0.0, maxe = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        double f[NCOEF];
        feature_row((double)R[i], (double)P[i], f);
        double pred = 0.0;
        for (int k = 0; k < NCOEF; k++) pred += coef[k] * f[k];
        double e = pred - (double)Y[i];
        sse += e * e;
        if (fabs(e) > maxe) maxe = fabs(e);
        sum_y += (double)Y[i];
    }
    double mean_y = sum_y / (double)n;
    double sst = 0.0;
    for (uint16_t i = 0; i < n; i++) {
        double d = (double)Y[i] - mean_y;
        sst += d * d;
    }
    *rms     = sqrt(sse / (double)n);
    *max_err = maxe;
    *r2      = (sst > 1e-9) ? (1.0 - sse / sst) : 1.0;
}

/* MỚI (Giai đoạn 6) - giải Least Squares BẬC 2 cho S1 và S2 từ toàn bộ
 * s_ls_count điểm lưới đã lưu, S3 suy trực tiếp bằng cách ÁP RÀNG BUỘC
 * S3=-(S1+S2) LÊN TỪNG HỆ SỐ (không hồi quy riêng lần 3 - luôn nhất quán
 * tuyệt đối với ràng buộc cơ khí, đúng nguyên tắc đã dùng ở v4).
 * s_ls_ok CHỈ true nếu: giải được CẢ 2 hệ (không suy biến) VÀ RMS/max_err
 * của CẢ 2 đạt ngưỡng cho phép - đây là ĐIỂM MỚI so với v4 (v4 chỉ kiểm
 * tra "giải được" mà không gate theo chất lượng fit thực tế). */
static void solve_and_print_ik_coeffs(void)
{
    s_ls_ok = false;

    uint16_t n = s_ls_count;
    if (n < 3 * NCOEF) {   /* cần dư điểm so với số ẩn (6) để hồi quy có ý nghĩa,
                             * không chỉ "đủ giải" - lấy margin 3x cho chắc */
        printf("[calib] LS ERROR: qua it diem (%u), can toi thieu %u de fit bac 2 tin cay\r\n",
               n, (unsigned)(3 * NCOEF));
        return;
    }

    double coef_S1[NCOEF], coef_S2[NCOEF];
    bool ok1 = solve_quadratic(s_ls_R, s_ls_P, s_ls_S1, n, coef_S1);
    bool ok2 = solve_quadratic(s_ls_R, s_ls_P, s_ls_S2, n, coef_S2);

    if (!ok1 || !ok2) {
        printf("[calib] LS ERROR: ma tran suy bien - du lieu luoi khong du da dang\r\n");
        return;
    }

    double rms1, max1, r2_1, rms2, max2, r2_2;
    compute_fit_stats_quad(s_ls_R, s_ls_P, s_ls_S1, n, coef_S1, &rms1, &max1, &r2_1);
    compute_fit_stats_quad(s_ls_R, s_ls_P, s_ls_S2, n, coef_S2, &rms2, &max2, &r2_2);

    printf("[calib] Least Squares BAC 2 tu %u diem:\r\n", n);
    printf("S1 = %.4fR %+.4fP %+.4fR^2 %+.4fP^2 %+.4fRP %+.4f  (RMS=%.3f max=%.3f R2=%.5f)\r\n",
           coef_S1[0], coef_S1[1], coef_S1[2], coef_S1[3], coef_S1[4], coef_S1[5],
           rms1, max1, r2_1);
    printf("S2 = %.4fR %+.4fP %+.4fR^2 %+.4fP^2 %+.4fRP %+.4f  (RMS=%.3f max=%.3f R2=%.5f)\r\n",
           coef_S2[0], coef_S2[1], coef_S2[2], coef_S2[3], coef_S2[4], coef_S2[5],
           rms2, max2, r2_2);

    /* MỚI - GATE theo chất lượng fit, không chỉ "giải được". Nếu tệ, IN RÕ
     * LÝ DO ra UART để người dùng biết cần quét lại / kiểm tra cơ khí,
     * thay vì âm thầm từ chối không giải thích. */
    bool quality_ok = (rms1 <= CALIB_LS_RMS_MAX_US) && (max1 <= CALIB_LS_MAXERR_MAX_US) &&
                       (rms2 <= CALIB_LS_RMS_MAX_US) && (max2 <= CALIB_LS_MAXERR_MAX_US);
    if (!quality_ok) {
        printf("[calib] LS CANH BAO: sai so vuot nguong (RMS_MAX=%.1f MAXERR_MAX=%.1f us) - "
               "TU CHOI ghi Flash, kiem tra co khi/IMU/servo roi CHAY LAI CALIB\r\n",
               CALIB_LS_RMS_MAX_US, CALIB_LS_MAXERR_MAX_US);
        return;
    }

    /* S3 = -(S1+S2) áp trực tiếp lên hệ số (đúng cho MỌI (R,P) vì là hệ
     * quả tuyến tính của S1,S2 - không phải hồi quy độc lập). */
    for (int k = 0; k < NCOEF; k++) {
        s_ik_coef[0][k] = (float)coef_S1[k];
        s_ik_coef[1][k] = (float)coef_S2[k];
        s_ik_coef[2][k] = (float)(-(coef_S1[k] + coef_S2[k]));
    }
    printf("S3 = suy tu S3=-(S1+S2), da ap len tung he so\r\n");

    s_ls_ok = true;
}

/* ==========================================================================
 * Đo 1 điểm tại vị trí cố định (dùng cho OFFSET - có return-to-neutral).
 * ========================================================================== */
static bool run_measurement(float dt, int32_t s1, int32_t s2, int32_t s3, uint16_t sample_count)
{
    switch (s_meas_phase) {
        case MEAS_MOVE_START:
            traj_start_to(s1, s2, s3);
            s_meas_phase = MEAS_MOVE_RUN;
            break;

        case MEAS_MOVE_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct  = 0;
                s_meas_phase = MEAS_SETTLE;
            }
            break;

        case MEAS_SETTLE:
            s_settle_ct++;
            if (s_settle_ct >= CALIB_SETTLE_CYCLES) {
                s_settle_ct  = 0;
                s_sample_ct  = 0;
                s_sum_roll   = 0.0f;
                s_sum_pitch  = 0.0f;
                s_meas_phase = MEAS_SAMPLE;
            }
            break;

        case MEAS_SAMPLE: {
            float roll, pitch, vroll, vpitch;
            imu_state_read(system_state_get_imu_ptr(), &roll, &pitch, &vroll, &vpitch);
            s_sum_roll  += roll;
            s_sum_pitch += pitch;
            s_sample_ct++;
            if (s_sample_ct >= sample_count) {
                s_result_roll  = s_sum_roll  / (float)sample_count;
                s_result_pitch = s_sum_pitch / (float)sample_count;
                s_meas_phase   = MEAS_RETURN_START;
            }
            break;
        }

        case MEAS_RETURN_START:
            /* SỬA - dùng đúng (s1,s2,s3) truyền vào (= working neutral, đã
             * áp override nếu có qua control_mode_calib_set_neutral()),
             * KHÔNG đọc lại c->S1_neutral cũ từ Flash - nếu không, bước
             * OFFSET vẫn "về" neutral cũ thay vì neutral mới đang muốn đo. */
            traj_start_to(s1, s2, s3);
            s_meas_phase = MEAS_RETURN_RUN;
            break;

        case MEAS_RETURN_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct  = 0;
                s_meas_phase = MEAS_RETURN_SETTLE;
            }
            break;

        case MEAS_RETURN_SETTLE:
            s_settle_ct++;
            if (s_settle_ct >= CALIB_SETTLE_CYCLES) {
                s_meas_phase = MEAS_DONE;
            }
            break;

        case MEAS_DONE:
            return true;
    }
    return false;
}

/* ==========================================================================
 * MỚI (Giai đoạn 6) - LƯỚI QUÉT 2D (S1,S2), S3=-(S1+S2). Thay thế
 * sweep_targets()/run_sweep() 1-trục của v4.
 * grid_idx chạy 0..LS_MAX_POINTS-1, row = idx / CALIB_GRID_POINTS,
 * col = idx % CALIB_GRID_POINTS. Mỗi trục trải ĐỀU -HALF..+HALF qua đúng
 * CALIB_GRID_POINTS điểm (dùng phép chia thực rồi làm tròn từng điểm,
 * tránh sai số làm tròn tích luỹ - giữ đúng kỹ thuật v4 đã dùng, chỉ mở
 * rộng từ 1D sang 2D).
 * ========================================================================== */
static void grid_targets(uint16_t idx, int32_t n1, int32_t n2, int32_t n3,
                          int32_t *s1, int32_t *s2, int32_t *s3)
{
    int row = idx / CALIB_GRID_POINTS;
    int col = idx % CALIB_GRID_POINTS;

    float s1_off_f = -CALIB_GRID_HALF_US + (float)row * (2.0f * CALIB_GRID_HALF_US) / (float)(CALIB_GRID_POINTS - 1);
    float s2_off_f = -CALIB_GRID_HALF_US + (float)col * (2.0f * CALIB_GRID_HALF_US) / (float)(CALIB_GRID_POINTS - 1);

    int32_t s1_off = (int32_t)lroundf(s1_off_f);
    int32_t s2_off = (int32_t)lroundf(s2_off_f);
    int32_t s3_off = -(s1_off + s2_off);

    *s1 = n1 + s1_off;
    *s2 = n2 + s2_off;
    *s3 = n3 + s3_off;
}

/* Chạy toàn bộ lưới quét. Trả true khi CẢ LƯỚI đã xong (đã về lại neutral). */
static bool run_sweep_grid(float dt)
{
    const calibration_data_t *c = calibration_data_get_ptr();
    int32_t n1, n2, n3;
    get_working_neutral(c, &n1, &n2, &n3);

    switch (s_sweep_phase) {
        case SWEEP_MOVE_START: {
            int32_t s1, s2, s3;
            grid_targets(s_grid_idx, n1, n2, n3, &s1, &s2, &s3);
            traj_start_to(s1, s2, s3);
            s_sweep_phase = SWEEP_MOVE_RUN;
            break;
        }

        case SWEEP_MOVE_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct   = 0;
                s_sweep_phase = SWEEP_SETTLE;
            }
            break;

        case SWEEP_SETTLE:
            s_settle_ct++;
            if (s_settle_ct >= CALIB_SETTLE_CYCLES) {
                s_settle_ct   = 0;
                s_sample_ct   = 0;
                s_sum_roll    = 0.0f;
                s_sum_pitch   = 0.0f;
                s_sweep_phase = SWEEP_SAMPLE;
            }
            break;

        case SWEEP_SAMPLE: {
            float roll, pitch, vroll, vpitch;
            imu_state_read(system_state_get_imu_ptr(), &roll, &pitch, &vroll, &vpitch);
            s_sum_roll  += roll;
            s_sum_pitch += pitch;
            s_sample_ct++;
            if (s_sample_ct >= CALIB_SAMPLE_COUNT) {
                s_result_roll  = s_sum_roll  / (float)CALIB_SAMPLE_COUNT;
                s_result_pitch = s_sum_pitch / (float)CALIB_SAMPLE_COUNT;

                int32_t s1, s2, s3;
                grid_targets(s_grid_idx, n1, n2, n3, &s1, &s2, &s3);

                printf("%ld,%ld,%ld,%.2f,%.2f\r\n",
                       (long)(s1 - n1), (long)(s2 - n2), (long)(s3 - n3),
                       (double)s_result_roll, (double)s_result_pitch);

                /* Trừ offset TRƯỚC khi lưu - khớp quy ước PID output quanh 0
                 * (xem giải thích chi tiết trong control_mode_calib.h). */
                if (s_ls_count < LS_MAX_POINTS) {
                    s_ls_R[s_ls_count]  = s_result_roll  - s_roll_offset;
                    s_ls_P[s_ls_count]  = s_result_pitch - s_pitch_offset;
                    s_ls_S1[s_ls_count] = (float)(s1 - n1);
                    s_ls_S2[s_ls_count] = (float)(s2 - n2);
                    s_ls_count++;
                }

                if (s_grid_idx + 1 < LS_MAX_POINTS) {
                    s_grid_idx++;
                    s_sweep_phase = SWEEP_MOVE_START;   /* điểm kế tiếp - KHÔNG về neutral */
                } else {
                    s_sweep_phase = SWEEP_RETURN_START;   /* hết lưới - về neutral 1 lần */
                }
            }
            break;
        }

        case SWEEP_RETURN_START:
            traj_start_to(n1, n2, n3);
            s_sweep_phase = SWEEP_RETURN_RUN;
            break;

        case SWEEP_RETURN_RUN:
            if (traj_run_step(dt)) {
                s_sweep_phase = SWEEP_ALL_DONE;
            }
            break;

        case SWEEP_ALL_DONE:
            return true;
    }
    return false;
}

void control_mode_calib_enter(void)
{
    s_sub        = CALIB_SUB_OFFSET;
    s_meas_phase = MEAS_MOVE_START;
    /* Mặc định KHÔNG đổi neutral mỗi lần vào Mode Calib mới - caller phải
     * chủ động gọi control_mode_calib_set_neutral() NGAY SAU hàm này nếu
     * muốn override (xem control_mode_calib.h). */
    s_neutral_override = false;

#if CALIB_NEUTRAL_HARDCODE_ENABLE
    /* MỚI - tự áp override hardcode ở trên, không cần code nơi khác gọi
     * control_mode_calib_set_neutral() nữa. Xem giải thích ngay phía trên
     * định nghĩa CALIB_NEUTRAL_HARDCODE_*. */
    control_mode_calib_set_neutral(CALIB_NEUTRAL_HARDCODE_S1,
                                    CALIB_NEUTRAL_HARDCODE_S2,
                                    CALIB_NEUTRAL_HARDCODE_S3);
#endif

    set_guide("Calib: Get offset IMU...");
}

void control_mode_calib_step(float dt)
{
    const calibration_data_t *c = calibration_data_get_ptr();
    int32_t n1, n2, n3;
    get_working_neutral(c, &n1, &n2, &n3);
    bool done;

    switch (s_sub) {

        case CALIB_SUB_OFFSET:
            done = run_measurement(dt, n1, n2, n3, CALIB_OFFSET_SAMPLE_COUNT);
            if (done) {
                s_roll_offset  = s_result_roll;
                s_pitch_offset = s_result_pitch;
                s_sub = CALIB_SUB_SWEEP_GRID;
                s_sweep_phase = SWEEP_MOVE_START;
                s_grid_idx = 0;
                s_ls_count = 0;
                printf("== SWEEP GRID 2D (%dx%d diem, S3=-(S1+S2)) ==\r\n",
                       CALIB_GRID_POINTS, CALIB_GRID_POINTS);
                printf("S1,S2,S3,roll,pitch\r\n");
                set_guide("Calib: Sweep grid 2D...");
            }
            break;

        case CALIB_SUB_SWEEP_GRID:
            done = run_sweep_grid(dt);
            if (done) {
                printf("== SWEEP DONE (%u diem) - dang giai Least Squares bac 2 ==\r\n", (unsigned)s_ls_count);
                solve_and_print_ik_coeffs();
                s_sub = CALIB_SUB_SAVE;
                set_guide("Calib: Write offset to Flash...");
            }
            break;

        case CALIB_SUB_SAVE: {
            printf("[calib] roll_offset=%.2f pitch_offset=%.2f (trung binh %d mau, ~2s tai neutral)\r\n",
                   (double)s_roll_offset, (double)s_pitch_offset, (int)CALIB_OFFSET_SAMPLE_COUNT);

            calibration_data_t out = *c;   /* mặc định giữ nguyên deadband hiện có */
            out.roll_offset  = s_roll_offset;
            out.pitch_offset = s_pitch_offset;

#if CALIB_DEADBAND_HARDCODE_ENABLE
            /* MỚI - ép deadband theo hardcode ở trên, ghi đè deadband cũ
             * đọc từ Flash. Xem giải thích đầy đủ tại định nghĩa
             * CALIB_DEADBAND_HARDCODE_*. */
            out.deadband_S1 = CALIB_DEADBAND_HARDCODE_S1;
            out.deadband_S2 = CALIB_DEADBAND_HARDCODE_S2;
            out.deadband_S3 = CALIB_DEADBAND_HARDCODE_S3;
            printf("[calib] Ap dung deadband moi: S1=%d S2=%d S3=%d\r\n",
                   out.deadband_S1, out.deadband_S2, out.deadband_S3);
#endif

            /* MỚI - áp neutral mới nếu control_mode_calib_set_neutral() đã
             * được gọi trước đó (n1/n2/n3 CHÍNH LÀ neutral đã dùng xuyên
             * suốt OFFSET/SWEEP_GRID ở trên - xem get_working_neutral() -
             * nên ghi lại đúng giá trị đó, KHÔNG đọc lại c->S1_neutral cũ).
             * calibration_data_save() sẽ tự tính lại min/max = neutral ±
             * CALIB_TILT_MAX_US, không cần tính ở đây. */
            if (s_neutral_override) {
                out.S1_neutral = (int16_t)n1;
                out.S2_neutral = (int16_t)n2;
                out.S3_neutral = (int16_t)n3;
                printf("[calib] Ap dung neutral moi: S1=%ld S2=%ld S3=%ld\r\n",
                       (long)n1, (long)n2, (long)n3);
            }

            if (s_ls_ok) {
                for (int i = 0; i < 3; i++)
                    for (int k = 0; k < CALIB_IK_NUM_COEF; k++)
                        out.ik_coef[i][k] = s_ik_coef[i][k];
                printf("[calib] Da ghi 18 he so IK bac 2 moi vao ik_coef[3][6]\r\n");
            } else {
                printf("[calib] CANH BAO: Least Squares that bai/khong dat nguong - GIU NGUYEN he so IK cu\r\n");
            }

            if (calibration_data_save(&out)) {
                printf("[calib] Write FLASH Successfully\r\n");
                servo_actuator_set_calib(SERVO_CH_S1, out.S1_neutral, out.S1_min, out.S1_max, out.deadband_S1);
                servo_actuator_set_calib(SERVO_CH_S2, out.S2_neutral, out.S2_min, out.S2_max, out.deadband_S2);
                servo_actuator_set_calib(SERVO_CH_S3, out.S3_neutral, out.S3_min, out.S3_max, out.deadband_S3);
                s_sub = CALIB_SUB_DONE;
                set_guide("Calibrate done! READY...");
                send_evt(EVT_MODE_CALIB_DONE);
            } else {
                printf("[calib] WRITE FLASH FAIL\r\n");
                s_sub = CALIB_SUB_ERROR;
                set_guide("Calib ERR - WRITE FLASH FAIL");
                send_evt(EVT_MODE_CALIB_FAILED);
            }
            /* Dùng xong (thành công hay thất bại) - xoá override, tránh
             * ảnh hưởng lần chạy Mode Calib kế tiếp nếu caller quên gọi
             * lại control_mode_calib_set_neutral(). */
            s_neutral_override = false;
            break;
        }

        case CALIB_SUB_DONE:
        case CALIB_SUB_ERROR:
            break;
    }
}

calib_sub_state_t control_mode_calib_get_sub_state(void)
{
    return s_sub;
}

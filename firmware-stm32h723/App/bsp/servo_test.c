/**
 * @file    servo_test.c
 * @brief   Test mode doc lap cho B2 (implementation).
 * @see     servo_test.h, PingpongTable_ProfessionalDesign muc B.2.6, B.2.8
 */
#include "servo_test.h"
#include "servo_actuator.h"
#include "trajectory.h"
#include "system_state.h"       /* imu_state_read() - THÊM Giai đoạn 3 */
#include "calibration_data.h"   /* min/max/neutral cho sweep - THÊM Giai đoạn 3 */
#include <stdio.h>   /* printf - can retarget qua UART/ITM da co san tu B1 */

#define TEST_DT   0.01f   /* 100Hz - PHAI khop voi nhip goi servo_test_step() */

/* Tinh tu datasheet HPS-2018 (khong tai, 7.4V):
 *   180 deg / 2000us = 0.09 deg/us
 *   60 deg / 0.14s    = 428.57 deg/s
 *   -> toc do ly thuyet toi da = 428.57 / 0.09 = 4762 us/s (KHONG TAI)
 * Dung 60-65% cho v_max de con margin khi co tai that (mat ban nghieng,
 * ma sat, quan tinh) - tranh dong dinh gan stall current (1.4-2.3A datasheet).
 * a_max KHONG co trong datasheet (chi co toc do quay on dinh, khong co
 * gia toc/ramp-time) - uoc luong tu thoi gian tang toc dien hinh servo
 * digital nho (~100ms de dat v_max): a_max = v_max / 0.1s.
 * BAT BUOC tinh chinh lai bang thuc nghiem (muc 4.3, B2_Servo_Actuator_Summary.md). */
#define SERVO_TEST_TRAJ_V_MAX   2800.0f   /* us/s - ~59% cua 4762 us/s ly thuyet */
#define SERVO_TEST_TRAJ_A_MAX  20000.0f   /* us/s^2 - uoc luong, chinh lai thuc nghiem */

typedef enum {
    TEST_HOLD_NEUTRAL = 0,
    TEST_ABSOLUTE_SWEEP,
    TEST_INCREMENTAL,
    TEST_TRAJECTORY_HOME
} test_phase_t;

static test_phase_t  s_phase      = TEST_HOLD_NEUTRAL;
static uint32_t       s_phase_tick = 0;
static trajectory_t   s_traj[SERVO_CH_COUNT];

/* Ten giai doan - chi dung de in log cho de doc */
static const char *test_phase_name(test_phase_t p)
{
    switch (p) {
        case TEST_HOLD_NEUTRAL:     return "HOLD_NEUTRAL";
        case TEST_ABSOLUTE_SWEEP:   return "ABSOLUTE_SWEEP";
        case TEST_INCREMENTAL:      return "INCREMENTAL";
        case TEST_TRAJECTORY_HOME:  return "TRAJECTORY_HOME";
        default:                    return "?";
    }
}

void servo_test_init(void)
{
    servo_actuator_init();
    s_phase      = TEST_HOLD_NEUTRAL;
    s_phase_tick = 0;
}

void servo_test_step(void)
{
    s_phase_tick++;

    switch (s_phase) {

    case TEST_HOLD_NEUTRAL:
        /* (1) Giu neutral 2s -> xac nhan ca 3 servo ve dung 1500us, khong rung */
        if (s_phase_tick == 1) {
            servo_actuator_set_target(SERVO_CH_S1, 1500);
            servo_actuator_set_target(SERVO_CH_S2, 1500);
            servo_actuator_set_target(SERVO_CH_S3, 1500);
        }
        if (s_phase_tick > 200) {           /* 200 * 10ms = 2s */
            s_phase = TEST_ABSOLUTE_SWEEP;
            s_phase_tick = 0;
        }
        break;

    case TEST_ABSOLUTE_SWEEP:
        /* (2) API vi tri tuyet doi - quet min/max/neutral tren S1,
         * xac nhan slew-rate muot (khong giat) */
        if (s_phase_tick == 1)   servo_actuator_set_target(SERVO_CH_S1, 1000);
        if (s_phase_tick == 300) servo_actuator_set_target(SERVO_CH_S1, 2000);
        if (s_phase_tick == 600) servo_actuator_set_target(SERVO_CH_S1, 1500);
        if (s_phase_tick > 900) {           /* 9s cho 3 doan x 3s */
            s_phase = TEST_INCREMENTAL;
            s_phase_tick = 0;
        }
        break;

    case TEST_INCREMENTAL:
        /* (3) Incremental - cong don tung buoc nho tren S2, xac nhan
         * anti-windup khi cham bien max (khong "tran" ra ngoai) */
        if (s_phase_tick % 5 == 0) {
            servo_actuator_apply_delta(SERVO_CH_S2, +10);  /* day dan len max */
        }
        if (s_phase_tick > 400) {           /* 4s */
            s_phase = TEST_TRAJECTORY_HOME;
            s_phase_tick = 0;
        }
        break;

    case TEST_TRAJECTORY_HOME:
        /* (4) Trajectory - S3 dang dung yen o neutral (1500) tu TEST_HOLD_NEUTRAL,
         * chua bi dong cham gi o 2 pha truoc (chi dong S1, S2). De co quang duong
         * ma quan sat trajectory, buoc 1: day S3 ra 1000 bang set_target() (di
         * qua duong position thuong, khong phai trajectory). Buoc 2: doc lai VI
         * TRI THAT (khong hard-code "from") roi moi bat dau trajectory VE HOME. */
        if (s_phase_tick == 1) {
            servo_actuator_set_target(SERVO_CH_S3, 1000);   /* lech ra khoi neutral truoc */
        }
        if (s_phase_tick == 100) {   /* ~1s, du de S3 toi han 1000 voi slew-rate hien tai */
            int32_t s1, s2, s3;
            servo_actuator_get_local(&s1, &s2, &s3);
            trajectory_start(&s_traj[SERVO_CH_S3], (float)s3, 1500.0f,
                              SERVO_TEST_TRAJ_V_MAX, SERVO_TEST_TRAJ_A_MAX);
        }
        if (s_phase_tick > 100) {
            /* Xac nhan dung profile hinh thang (accel -> cruise -> decel).
             * SUA: trajectory_update() doi chu ky (nhan them dt + tra ve
             * qua con tro x_ref/v_ref/a_ref, thay vi return float truc
             * tiep) - xem trajectory.h ban moi. */
            float sp;
            trajectory_update(&s_traj[SERVO_CH_S3], TEST_DT, &sp, NULL, NULL);
            servo_actuator_set_target(SERVO_CH_S3, (int32_t)sp);
        }
        if (s_phase_tick > 400) {           /* du du cho ca dich chuyen + trajectory */
            s_phase = TEST_HOLD_NEUTRAL;
            s_phase_tick = 0;
        }
        break;
    }

    servo_actuator_step(TEST_DT);   /* luon goi cuoi cung, dung 1 lan moi chu ky */

    /* Debug print moi 0.5s - cung pattern imu_debug_print_dma_counters o B1 */
    if (s_phase_tick % 50 == 0) {
        int32_t s1, s2, s3;
        servo_actuator_get_local(&s1, &s2, &s3);
        printf("[servo_test] phase=%-16s S1=%ld S2=%ld S3=%ld\r\n",
               test_phase_name(s_phase), (long)s1, (long)s2, (long)s3);
    }
}

/* ==========================================================================
 * PHẦN THÊM CHO GIAI ĐOẠN 3 (B6) — SERVO_TEST_MODE_DEADBAND_SCAN /
 * MANUAL_STEP / SWEEP_LOG. Toàn bộ static bên dưới TÁCH RIÊNG khỏi state
 * (s_phase/s_phase_tick/s_traj) của 4-phase B2 ở trên - 2 bộ hoàn toàn độc
 * lập, không dùng chung biến, để không phá code B2 cũ.
 * ========================================================================== */

/* Hằng số khởi điểm theo B6_Control.md mục 4 - PHẢI tinh chỉnh thực nghiệm.
 * SERVO_TEST_MODE_DEADBAND_SCAN đã BỎ (Giai đoạn 4 - cảm biến nhiễu, đo
 * không chính xác) - các hằng số SERVO_TEST_DEADBAND_* cũng xoá theo. */
#define SERVO_TEST_SWEEP_STEP_US             20
#define SERVO_TEST_SWEEP_SETTLE_CYCLES       10

static servo_test_mode_t s_mode      = SERVO_TEST_MODE_LEGACY_B2;
static bool               s_running  = false;
static bool               s_done     = false;
static float               s_elapsed_ms = 0.0f;

/* Danh sách kênh cần test trong phiên hiện tại (servo_ch=0 -> cả 3 theo thứ
 * tự S1->S2->S3; servo_ch=1/2/3 -> chỉ 1 phần tử). */
static servo_ch_t s_ch_list[SERVO_CH_COUNT];
static uint8_t     s_ch_count = 0;
static uint8_t     s_ch_pos   = 0;      /* index đang test trong s_ch_list */

/* ---- Sweep log: sub-phase nội bộ (SỬA Giai đoạn 4) ----
 * PHIÊN BẢN MỚI: quét ĐỒNG THỜI cả 3 servo để giữ đúng ràng buộc cơ khí
 * bắt buộc "(S1-S1n)+(S2-S2n)+(S3-S3n)=0" tại MỌI thời điểm (không chỉ 2
 * đầu dải) - tránh hỏng cơ khí do chỉ 1 servo chịu tải lệch (theo file
 * "Giai đoạn 4..." mục cuối). Lần lượt chọn "servo chính" (primary) là
 * S1->S2->S3; khi primary quét t từ -CALIB_TILT_MAX_US đến +CALIB_TILT_MAX_US,
 * 2 servo còn lại LUÔN bù ngược -t/2 để giữ tổng = 0 tại mọi điểm giữa
 * đường quét, không chỉ 2 đầu. */
typedef enum {
    SWEEP_PHASE_SCAN = 0,
    SWEEP_PHASE_RETURN_NEUTRAL,
} sweep_phase_t;

static sweep_phase_t s_sweep_phase     = SWEEP_PHASE_SCAN;
static uint16_t        s_sweep_settle_ct = 0;
static float             s_sweep_t         = 0.0f;   /* -CALIB_TILT_MAX_US .. +CALIB_TILT_MAX_US */

/* ---- Manual step: kênh đang chỉnh tay ---- */
static uint8_t s_manual_channel = 1;   /* 1=S1,2=S2,3=S3 - mặc định S1 */

void servo_test_log_csv_header(void)
{
    printf("t_ms,S1_us,S2_us,S3_us,roll_deg,pitch_deg\r\n");
}

void servo_test_log_csv_row(uint32_t t_ms)
{
    int32_t s1, s2, s3;
    float roll, pitch, vroll, vpitch;
    servo_actuator_get_local(&s1, &s2, &s3);
    imu_state_read(system_state_get_imu_ptr(), &roll, &pitch, &vroll, &vpitch);
    printf("%lu,%ld,%ld,%ld,%.2f,%.2f\r\n",
           (unsigned long)t_ms, (long)s1, (long)s2, (long)s3, (double)roll, (double)pitch);
}

static void build_channel_list(uint8_t servo_ch)
{
    if (servo_ch == 0) {
        s_ch_list[0] = SERVO_CH_S1;
        s_ch_list[1] = SERVO_CH_S2;
        s_ch_list[2] = SERVO_CH_S3;
        s_ch_count   = 3;
    } else {
        s_ch_list[0] = (servo_ch_t)(servo_ch - 1);   /* servo_ch 1/2/3 -> SERVO_CH_S1/S2/S3 (0-based) */
        s_ch_count   = 1;
    }
    s_ch_pos = 0;
}

void servo_test_start(servo_test_mode_t mode, uint8_t servo_ch)
{
    s_mode       = mode;
    s_running    = true;
    s_done       = false;
    s_elapsed_ms = 0.0f;

    build_channel_list(servo_ch);

    s_sweep_phase      = SWEEP_PHASE_SCAN;
    s_sweep_settle_ct  = 0;
    s_sweep_t          = -(float)CALIB_TILT_MAX_US;

    s_manual_channel = (servo_ch >= 1 && servo_ch <= 3) ? servo_ch : 1;

    servo_test_log_csv_header();
}

void servo_test_stop(void)
{
    s_running = false;
}

bool servo_test_is_done(void)
{
    return s_done;
}

void servo_test_manual_select_channel(uint8_t servo_ch)
{
    if (servo_ch >= 1 && servo_ch <= 3) s_manual_channel = servo_ch;
}

void servo_test_manual_adjust(int16_t delta_us)
{
    servo_ch_t ch = (servo_ch_t)(s_manual_channel - 1);
    servo_actuator_apply_delta(ch, delta_us);
    /* Log ngay dòng CSV ứng với lệnh vừa bấm - servo_actuator_step() ở cuối
     * servo_test_step_dt() sẽ áp dụng slew-rate dần, nên giá trị S_us log
     * ra có thể chưa "tới" target ngay dòng này, đúng ý nghĩa "quan sát quá
     * trình di chuyển" thay vì chỉ điểm cuối. */
    servo_test_log_csv_row((uint32_t)s_elapsed_ms);
}

/* ---- Sweep log step (SỬA Giai đoạn 4 - quét đồng thời 3 servo) ---- */

static void sweep_log_step(void)
{
    if (s_ch_pos >= s_ch_count) { s_done = true; s_running = false; return; }

    servo_ch_t primary = s_ch_list[s_ch_pos];   /* 0=S1,1=S2,2=S3 - servo chính đang quét t */
    const calibration_data_t *c = calibration_data_get_ptr();
    int32_t neutral[3] = { c->S1_neutral, c->S2_neutral, c->S3_neutral };

    switch (s_sweep_phase) {

        case SWEEP_PHASE_SCAN:
            if (s_sweep_settle_ct == 0) {
                /* Servo chính: neutral + t. 2 servo còn lại: neutral - t/2 -
                 * đảm bảo tổng offset luôn = 0 tại MỌI t (không chỉ 2 đầu):
                 *   t + (-t/2) + (-t/2) = 0  với mọi t. */
                int32_t tgt[3];
                for (uint8_t i = 0; i < 3; i++) {
                    tgt[i] = (i == primary)
                             ? (neutral[i] + (int32_t)s_sweep_t)
                             : (neutral[i] - (int32_t)(s_sweep_t / 2.0f));
                }
                servo_actuator_set_target(SERVO_CH_S1, tgt[0]);
                servo_actuator_set_target(SERVO_CH_S2, tgt[1]);
                servo_actuator_set_target(SERVO_CH_S3, tgt[2]);
            }
            s_sweep_settle_ct++;
            if (s_sweep_settle_ct >= SERVO_TEST_SWEEP_SETTLE_CYCLES) {
                servo_test_log_csv_row((uint32_t)s_elapsed_ms);
                s_sweep_settle_ct = 0;

                if (s_sweep_t >= (float)CALIB_TILT_MAX_US) {
                    s_sweep_phase = SWEEP_PHASE_RETURN_NEUTRAL;
                } else {
                    s_sweep_t += (float)SERVO_TEST_SWEEP_STEP_US;
                    if (s_sweep_t > (float)CALIB_TILT_MAX_US) {
                        s_sweep_t = (float)CALIB_TILT_MAX_US;   /* chốt đúng biên, tránh vượt do bước lẻ */
                    }
                }
            }
            break;

        case SWEEP_PHASE_RETURN_NEUTRAL:
            if (s_sweep_settle_ct == 0) {
                servo_actuator_set_target(SERVO_CH_S1, neutral[0]);
                servo_actuator_set_target(SERVO_CH_S2, neutral[1]);
                servo_actuator_set_target(SERVO_CH_S3, neutral[2]);
            }
            s_sweep_settle_ct++;
            if (s_sweep_settle_ct >= SERVO_TEST_SWEEP_SETTLE_CYCLES) {
                s_sweep_settle_ct = 0;
                s_sweep_phase     = SWEEP_PHASE_SCAN;
                s_sweep_t         = -(float)CALIB_TILT_MAX_US;   /* reset cho servo chính kế tiếp */
                s_ch_pos++;
            }
            break;
    }
}

void servo_test_step_dt(float dt)
{
    if (!s_running) return;
    s_elapsed_ms += dt * 1000.0f;

    switch (s_mode) {
        case SERVO_TEST_MODE_MANUAL_STEP:   /* chỉ chờ lệnh servo_test_manual_adjust(),
                                                không tự làm gì thêm mỗi chu kỳ */ break;
        case SERVO_TEST_MODE_SWEEP_LOG:     sweep_log_step();     break;
        default: break;
    }

    servo_actuator_step(dt);
}

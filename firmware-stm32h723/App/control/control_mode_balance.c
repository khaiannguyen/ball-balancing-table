#include "control_mode_balance.h"
#include "control_ball_common.h"
#include "trajectory.h"
#include "servo_actuator.h"
#include "system_state.h"
#include <stdbool.h>
#include <math.h>
#include "main.h"
#include <stdio.h>

/* ============================================================================
 * Trajectory Engine acceleration-continuous o KHONG GIAN Roll/Pitch/Height,
 * TRUOC khi vao IK (khac slew-rate Tang 2, hoat dong o khong gian truc
 * S1/S2/S3 SAU IK - day la lop LOC THEM, KHONG thay the Tang 2).
 *
 * Jetson gui setpoint LIEN TUC tan so cao (khong phai lenh roi rac kieu
 * "di toi 1 diem roi dung"), nen cach dung trajectory_state_t o day khac
 * cach dung kinh dien (Home/Calib): MOI KHI setpoint moi khac setpoint
 * dang chay do QUA epsilon (TARGET_EPSILON_*, xem duoi), trajectory duoc
 * REPLAN ngay TU TRANG THAI (x,v,a) HIEN TAI - khong reset ve v=0 (diem
 * sua cot loi, xem trajectory.h) va khong doi trajectory cu chay xong.
 * Neu Jetson doi setpoint moi chu ky (binh thuong), dieu nay xay ra MOI
 * CHU KY - do la CHU DICH thiet ke: hoat dong giong 1 bo loc gioi han
 * van toc/gia toc lien tuc hon la 1 "chuyen di" hoan chinh.
 * ========================================================================== */

/* Servo dang dung: 40kg.cm, 0.085s/60 do (khong tai)
 *   -> toc do goc ly thuyet toi da = 60/0.085 = 705.88 do/s
 * Roll/Pitch: dung DUNG gia tri lon nhat nay (khong bot margin) cho lop
 * loc mem nay, vi Tang 2 (co margin an toan rieng, xem
 * SERVO_SLEW_MAX_US_PER_S trong servo_actuator.h) van la lop chan cung
 * cuoi cung phia sau. */
#define SERVO_V_MAX_THEORETICAL_DEG_S   (60.0f / 0.085f)   /* = 705.88 do/s */

/* Roll/Pitch: KHONG co he so quy doi co dinh do<->us (IK la da thuc BAC 2,
 * do nhay S/do thay doi theo diem lam viec) - dung TAM toc do goc ly
 * thuyet servo cho V_max, va uoc luong dat V_max trong ~15ms cho A_max
 * (dung tinh than ke hoach muc 1, CHUA co so lieu gia toc that tu
 * datasheet). TODO: xac nhan lai ca 2 bang thuc nghiem (quan sat S1/S2/S3
 * that khi Roll_d/Pitch_d doi nhanh - neu thay lien tuc bi Tang 2 slew-
 * rate cat/giat, giam gia tri xuong). */
#define TRAJ_ROLL_PITCH_V_MAX   (SERVO_V_MAX_THEORETICAL_DEG_S)     /* do/s */
#define TRAJ_ROLL_PITCH_A_MAX   (TRAJ_ROLL_PITCH_V_MAX / 0.015f)    /* do/s^2 */

/* Height: quy doi CHINH XAC qua HEIGHT_K_US_PER_MM - PHAI GIONG HET
 * K_US_PER_MM (static, khong public) trong control_ball_common.c. Neu sua
 * 1 ben thi PHAI sua ben kia theo, khong de lech nhau (ca 2 deu se duoc
 * do lai tren servo moi, xem ke hoach muc 5.1). */
#define HEIGHT_K_US_PER_MM      (290.0f / 13.0f)            /* us/mm */

/* SUA: khong hardcode 6500.0f/us-tran rieng nua - tham chieu THANG toi
 * SERVO_SLEW_MAX_US_PER_S / SERVO_ACCEL_MAX_US_PER_S2 (servo_actuator.h,
 * nguon chan ly duy nhat cho servo 40kg.cm), quy doi qua HEIGHT_K_US_PER_MM
 * - tu dong dong bo khi tune lai gia tri goc, khong can sua 2 noi. */
#define TRAJ_HEIGHT_V_MAX   (SERVO_SLEW_MAX_US_PER_S  / HEIGHT_K_US_PER_MM)  /* mm/s */
#define TRAJ_HEIGHT_A_MAX   (SERVO_ACCEL_MAX_US_PER_S2 / HEIGHT_K_US_PER_MM) /* mm/s^2 */

/* Chong target noise: setpoint moi tu Jetson chi kich hoat replan() neu
 * lech qua epsilon nay - tranh tinh lai quy dao cho nhieu rung nho tu
 * luong CAN/vision. TODO: xac dinh gia tri cu the sau khi log noise thuc
 * te (hien dang dat tam, uu tien an toan - qua nho con hon qua lon). */
#define TARGET_EPSILON_DEG   0.05f
#define TARGET_EPSILON_MM    0.05f

/* Guard dt: neu 1 tick bi block qua lau (vd task khac chiem CPU), dt lon
 * bat thuong se lam trajectory "nhay" 1 doan xa chi trong 1 buoc update -
 * bo qua nhung chu ky nay thay vi update trajectory voi dt sai lech. */
#define DT_MAX   0.5f   /* giay - TODO: xac nhan gia tri hop ly thuc te */

/* Co Ball ON/OFF - xem TODO trong control_mode_balance.h ve viec thay
 * bang nguon "su that" khac neu da co san. */
static bool s_ball_on = false;

/* Trajectory + trang thai noi suy hien tai cho Roll/Pitch/Height. s_*_cur
 * la vi tri THAM CHIEU (command state) dang noi suy - dung lam diem
 * "x0" khi replan (v0/a0 doc THANG tu chinh trajectory_state_t, khong
 * can luu rieng). s_*_target dung de phat hien setpoint moi tu Jetson
 * (so sanh co epsilon voi setpoint dang chay). */
static trajectory_state_t s_traj_roll;
static trajectory_state_t s_traj_pitch;
static trajectory_state_t s_traj_height;
static float s_roll_cur   = 0.0f, s_pitch_cur   = 0.0f, s_height_cur   = 0.0f;
static float s_roll_target = 0.0f, s_pitch_target = 0.0f, s_height_target = 0.0f;
static bool  s_traj_inited = false;   /* false = chua co trajectory nao replan
                                          lan nao - lan dau tien (hoac sau khi
                                          vao lai mode) se replan bat buoc,
                                          khong dua vao du lieu s_traj_*.v/a cu */

void control_mode_balance_set_ball_on(bool on)
{
    s_ball_on = on;
}

bool control_mode_balance_get_ball_on(void)
{
    return s_ball_on;
}

void control_mode_balance_enter(void)
{
    /* Vao mode luon o Ball:OFF truoc cho an toan - nguoi dung phai chu
     * dong bat Ball:ON (qua UI/nut) sau khi da o trong mode, tranh truong
     * hop servo giat dot ngot theo Roll_d/Pitch_d/Height_d cu con sot lai
     * trong setpoint tu lan chay truoc. */
    s_ball_on = false;

    /* Reset toan bo trajectory + trang thai noi suy - tranh trajectory
     * "dang chay do" tu lan chay truoc con sot lai gay giat. */
    s_traj_roll   = (trajectory_state_t){0};
    s_traj_pitch  = (trajectory_state_t){0};
    s_traj_height = (trajectory_state_t){0};
    s_roll_cur = s_pitch_cur = s_height_cur = 0.0f;
    s_roll_target = s_pitch_target = s_height_target = 0.0f;
    s_traj_inited = false;
}

void control_mode_balance_step(float dt)
{
    /* Guard dt: dt <=0 (loi doc timer) hoac qua lon (tick bi block lau) -
     * bo qua chu ky nay hoan toan, KHONG update trajectory nao ca, giu
     * nguyen state hien tai cho tick sau voi dt binh thuong. */
    if (dt <= 0.0f || dt > DT_MAX) {
        return;
    }

    static bool last_ball_on = false;   /* chi in khi doi trang thai */

    bool camera_ok = camera_state_is_ok();

    if (s_ball_on != last_ball_on) {
        printf("Ball:ON = %d (camera_ok=%d)\r\n", s_ball_on, camera_ok);
        last_ball_on = s_ball_on;
    }

    if (s_ball_on && camera_ok) {
        /* ---- Ball:ON - nhan Roll_d/Pitch_d/Height_d tu Jetson, cho qua
         * Trajectory Engine (Tang 3) truoc khi vao IK. */
        setpoint_t sp;
        if (setpoint_get(&sp)) {
            /* Setpoint moi lech qua epsilon so voi setpoint dang chay do
             * (hoac lan dau sau khi vao mode) -> replan CA 3 trajectory
             * NGAY TU TRANG THAI (x,v,a) HIEN TAI - khong reset v=0, khong
             * doi trajectory cu chay xong (uu tien mem hon bam sat tuc
             * thi, dung tinh than thiet ke). Neu Jetson doi setpoint moi
             * chu ky (binh thuong), dieu nay xay ra MOI CHU KY - CHU DICH,
             * khong phai loi. */
            if (!s_traj_inited ||
                fabsf(sp.Roll_d   - s_roll_target)   >= TARGET_EPSILON_DEG ||
                fabsf(sp.Pitch_d  - s_pitch_target)  >= TARGET_EPSILON_DEG ||
                fabsf(sp.Height_d - s_height_target) >= TARGET_EPSILON_MM) {

                trajectory_replan(&s_traj_roll, s_roll_cur,
                                   s_traj_roll.v, s_traj_roll.a,
                                   sp.Roll_d, TRAJ_ROLL_PITCH_V_MAX, TRAJ_ROLL_PITCH_A_MAX);
                trajectory_replan(&s_traj_pitch, s_pitch_cur,
                                   s_traj_pitch.v, s_traj_pitch.a,
                                   sp.Pitch_d, TRAJ_ROLL_PITCH_V_MAX, TRAJ_ROLL_PITCH_A_MAX);
                trajectory_replan(&s_traj_height, s_height_cur,
                                   s_traj_height.v, s_traj_height.a,
                                   sp.Height_d, TRAJ_HEIGHT_V_MAX, TRAJ_HEIGHT_A_MAX);

                s_roll_target   = sp.Roll_d;
                s_pitch_target  = sp.Pitch_d;
                s_height_target = sp.Height_d;
                s_traj_inited   = true;
            }
        }
        /* Neu miss mutex (timeout) - KHONG replan, KHONG doi target, chi
         * don gian khong co setpoint moi chu ky nay. Trajectory dang chay
         * do (neu co) van tiep tuc noi suy binh thuong o duoi. */

        trajectory_update(&s_traj_roll,   dt, &s_roll_cur,   NULL, NULL);
        trajectory_update(&s_traj_pitch,  dt, &s_pitch_cur,  NULL, NULL);
        trajectory_update(&s_traj_height, dt, &s_height_cur, NULL, NULL);

        control_ball_apply_rph(s_roll_cur, s_pitch_cur, s_height_cur);

    } else {
        /* ---- Ball:OFF, hoac mat ket noi Jetson (failsafe) ----
         * Dung LAI CHINH trajectory engine, replan ve target=0 (Roll=
         * Pitch=Height=0, ban phang) thay vi reset s_*_cur ve 0 tuc thi -
         * thong nhat 1 engine cho ca 2 nhanh ON va OFF, khong tu viet
         * logic braking rieng o day. Dung Vmax/Amax BINH THUONG (day
         * KHONG PHAI duong E-stop khan cap - xem control_mode_balance.h). */
        if (!s_traj_inited ||
            fabsf(s_roll_target)   >= TARGET_EPSILON_DEG ||
            fabsf(s_pitch_target)  >= TARGET_EPSILON_DEG ||
            fabsf(s_height_target) >= TARGET_EPSILON_MM) {

            trajectory_replan(&s_traj_roll, s_roll_cur,
                               s_traj_roll.v, s_traj_roll.a,
                               0.0f, TRAJ_ROLL_PITCH_V_MAX, TRAJ_ROLL_PITCH_A_MAX);
            trajectory_replan(&s_traj_pitch, s_pitch_cur,
                               s_traj_pitch.v, s_traj_pitch.a,
                               0.0f, TRAJ_ROLL_PITCH_V_MAX, TRAJ_ROLL_PITCH_A_MAX);
            trajectory_replan(&s_traj_height, s_height_cur,
                               s_traj_height.v, s_traj_height.a,
                               0.0f, TRAJ_HEIGHT_V_MAX, TRAJ_HEIGHT_A_MAX);

            s_roll_target = s_pitch_target = s_height_target = 0.0f;
            s_traj_inited = true;
        }

        trajectory_update(&s_traj_roll,   dt, &s_roll_cur,   NULL, NULL);
        trajectory_update(&s_traj_pitch,  dt, &s_pitch_cur,  NULL, NULL);
        trajectory_update(&s_traj_height, dt, &s_height_cur, NULL, NULL);

        control_ball_apply_rph(s_roll_cur, s_pitch_cur, s_height_cur);
    }
}

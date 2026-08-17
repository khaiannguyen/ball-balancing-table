/**
 * @file    trajectory.c
 * @brief   Tang 3 - Trajectory generation, acceleration-continuous replan
 *          (implementation).
 * @see     trajectory.h, ke_hoach_sua_trajectory_actuator_ik_servo_moi.md muc 3
 */
#include "trajectory.h"
#include <math.h>
#include <string.h>

/* Nguong sai so - tranh so sanh float bang '==' truc tiep. Don vi giong
 * don vi cua x/v truyen vao (do, mm, us...) nen chi mang tinh "du nho" cho
 * moi truc dang dung trong he thong nay, khong can chinh xac tuyet doi. */
#define TRAJ_X_EPS   1e-4f
#define TRAJ_V_EPS   1e-4f

/* Cong thuc dong cho 1 doan gia toc hang so:
 *   x(t) = x0 + v0*t + 0.5*a*t^2
 *   v(t) = v0 + a*t                                          */
static void eval_const_accel(float x0, float v0, float a, float t,
                              float *x, float *v)
{
    *x = x0 + v0 * t + 0.5f * a * t * t;
    *v = v0 + a * t;
}

/* Xay toi da 3 doan ACCEL/CRUISE/DECEL vao tr->seg[] bat dau tu base_idx,
 * xuat phat tu (x_start, v_signed) chay ve target theo huong dir (+1/-1).
 * v_signed PHAI cung huong voi dir (hoac ~0) - truong hop nguoc huong da
 * duoc xu ly bang 1 doan BRAKE rieng truoc do (build_brake_segment()).
 * Tra ve so doan MOI duoc them (0..3).
 *
 * Toan bo dung 1 cong thuc duy nhat cho ca 2 truong hop v0=0 (kieu cu) va
 * v0!=0 (moi): goi s = thanh phan van toc theo dir, d = quang duong con
 * lai (>=0). Neu chi accel roi decel thang ve 0 (khong cham v_max):
 *     d = (v_peak^2 - s^2)/(2a) + v_peak^2/(2a)
 *  => v_peak^2 = a*d + s^2/2
 * Neu v_peak > v_max -> profile hinh thang (ACCEL toi v_max, CRUISE, DECEL). */
static int build_ramp_segments(trajectory_state_t *tr, int base_idx,
                                float x_start, float v_signed, float target,
                                float dir, float v_max, float a_max)
{
    float d = (target - x_start) * dir;
    if (d < TRAJ_X_EPS) return 0;   /* da toi dich theo huong nay */

    float s = v_signed * dir;
    if (s < 0.0f) s = 0.0f;         /* phong thu - khong nen xay ra o day */
    if (s > v_max) s = v_max;       /* qua toc do cho phep (hiem, vd v_max
                                        bi ha xuong giua luc dang chay) -
                                        cat bot, TODO: xem ke hoach muc 6 */

    float v_peak_sq = a_max * d + 0.5f * s * s;
    if (v_peak_sq < s * s) v_peak_sq = s * s;  /* d qua nho so voi s hien
        tai - khong du quang duong de phanh dung luc; danh v_peak=s (DECEL
        ngay lap tuc), chap nhan co the vuot dich mot chut - truong hop
        hiem, xem ke hoach muc 6 */
    float v_peak = sqrtf(v_peak_sq);
    if (v_peak > v_max) v_peak = v_max;

    float a_signed = a_max * dir;
    float t_accel  = (v_peak > s) ? (v_peak - s) / a_max : 0.0f;
    float t_decel  = v_peak / a_max;

    int   idx   = base_idx;
    float seg_x = x_start, seg_v = v_signed;

    if (t_accel > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_ACCEL;
        tr->seg[idx].duration = t_accel;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;
        tr->seg[idx].a_const  = a_signed;
        float xe, ve;
        eval_const_accel(seg_x, seg_v, a_signed, t_accel, &xe, &ve);
        seg_x = xe; seg_v = ve;
        idx++;
    }

    float d_so_far = (seg_x - x_start) * dir;
    float d_decel  = (v_peak * v_peak) / (2.0f * a_max);
    float d_cruise = d - d_so_far - d_decel;
    if (d_cruise < 0.0f) d_cruise = 0.0f;
    float t_cruise = (v_peak > TRAJ_V_EPS) ? (d_cruise / v_peak) : 0.0f;

    if (t_cruise > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_CRUISE;
        tr->seg[idx].duration = t_cruise;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;   /* hang so = v_peak*dir suot doan */
        tr->seg[idx].a_const  = 0.0f;
        float xe, ve;
        eval_const_accel(seg_x, seg_v, 0.0f, t_cruise, &xe, &ve);
        seg_x = xe; seg_v = ve;
        idx++;
    }

    if (t_decel > TRAJ_V_EPS) {
        tr->seg[idx].type     = TRAJ_PHASE_DECEL;
        tr->seg[idx].duration = t_decel;
        tr->seg[idx].x_start  = seg_x;
        tr->seg[idx].v_start  = seg_v;
        tr->seg[idx].a_const  = -a_signed;   /* nguoc dau ACCEL, ve 0 tai target */
        idx++;
    }

    return idx - base_idx;
}

/* Doan BRAKE tuong minh: v0 dang nguoc huong can di -> gia toc hang so
 * nguoc dau v0, dua v ve 0. Ghi vao tr->seg[base_idx], tra ve 1 neu co
 * them doan (0 neu |v0| qua nho, khong dang mot doan rieng). */
static int build_brake_segment(trajectory_state_t *tr, int base_idx,
                                float x_start, float v0, float a_max)
{
    if (fabsf(v0) < TRAJ_V_EPS) return 0;

    float dir0     = (v0 > 0.0f) ? 1.0f : -1.0f;   /* huong dang di truoc khi phanh */
    float t_brake  = fabsf(v0) / a_max;
    if (t_brake <= TRAJ_V_EPS) return 0;

    tr->seg[base_idx].type     = TRAJ_PHASE_BRAKE;
    tr->seg[base_idx].duration = t_brake;
    tr->seg[base_idx].x_start  = x_start;
    tr->seg[base_idx].v_start  = v0;
    tr->seg[base_idx].a_const  = -a_max * dir0;   /* nguoc huong v0 */
    return 1;
}

void trajectory_replan(trajectory_state_t *tr, float x0, float v0, float a0,
                        float target, float v_max, float a_max)
{
    (void)a0;   /* Giai doan 1 (acceleration-continuous) chua dung a0 - de
                   danh cho Phase B jerk-limited sau nay, xem trajectory.h */

    memset(tr, 0, sizeof(*tr));
    tr->target = target;
    tr->v_max  = v_max;
    tr->a_max  = a_max;
    tr->x = x0;
    tr->v = v0;
    tr->a = 0.0f;

    float dx = target - x0;

    if (fabsf(dx) < TRAJ_X_EPS && fabsf(v0) < TRAJ_V_EPS) {
        /* Da o dich, dung yen - khong tao doan nao (Buoc 1, truong hop
           dac biet: khong can di chuyen) */
        tr->seg_count = 0;
        tr->seg_index = 0;
        return;
    }

    float dir = (fabsf(dx) < TRAJ_X_EPS) ? ((v0 > 0.0f) ? 1.0f : -1.0f)
                                          : ((dx > 0.0f) ? 1.0f : -1.0f);

    int   n  = 0;
    float cx = x0, cv = v0;

    if (v0 * dir < -TRAJ_V_EPS) {
        /* Buoc 3 (ke hoach muc 3.3): v0 nguoc huong can di -> phanh truoc */
        int added = build_brake_segment(tr, n, cx, v0, a_max);
        if (added > 0) {
            trajectory_segment_t *b = &tr->seg[n];
            float xe, ve;
            eval_const_accel(b->x_start, b->v_start, b->a_const, b->duration, &xe, &ve);
            cx = xe; cv = ve;   /* ve ~0, con sai so float nho la binh thuong */
            n += added;
        }

        /* Sau khi phanh xong, huong toi target co the da doi (neu |v0|
           lon gay vuot qua diem target trong luc phanh) - tinh lai */
        dx = target - cx;
        if (fabsf(dx) < TRAJ_X_EPS) {
            tr->seg_count = n;
            tr->seg_index = 0;
            return;
        }
        dir = (dx > 0.0f) ? 1.0f : -1.0f;
    }

    /* Buoc 2 (ke hoach muc 3.3): v0 (hoac diem sau BRAKE) cung huong can
       di - xay ACCEL/CRUISE/DECEL, tu dong rut gon "tam giac" neu qua
       ngan (da xu ly ben trong build_ramp_segments, Buoc 4 tai su dung
       chung 1 cong thuc, khong viet rieng nhanh tam giac) */
    n += build_ramp_segments(tr, n, cx, cv, target, dir, v_max, a_max);

    tr->seg_count = n;
    tr->seg_index = 0;
}

void trajectory_update(trajectory_state_t *tr, float dt,
                        float *x_ref, float *v_ref, float *a_ref)
{
    if (dt < 0.0f) dt = 0.0f;   /* dt am khong hop le - khong lui thoi gian */

    float dt_left = dt;

    /* Di qua cac doan theo dt - vong lap de KHONG bo sot doan nao neu dt
       lon hon 1 doan (hiem trong van hanh binh thuong ~100Hz, nhung van
       phai dung, xem ke hoach muc 3.3). */
    while (tr->seg_index < tr->seg_count && dt_left > 0.0f) {
        trajectory_segment_t *seg = &tr->seg[tr->seg_index];
        float remain = seg->duration - tr->t_in_seg;
        if (remain < 0.0f) remain = 0.0f;

        if (dt_left < remain) {
            tr->t_in_seg += dt_left;
            dt_left = 0.0f;
        } else {
            dt_left -= remain;
            tr->seg_index++;
            tr->t_in_seg = 0.0f;
        }
    }

    if (tr->seg_index >= tr->seg_count) {
        /* Da xong tat ca doan - dung han tai target, KHONG ngoai suy them
           (tranh overshoot do sai so tich luy float) */
        tr->x = tr->target;
        tr->v = 0.0f;
        tr->a = 0.0f;
    } else {
        trajectory_segment_t *seg = &tr->seg[tr->seg_index];
        float x, v;
        eval_const_accel(seg->x_start, seg->v_start, seg->a_const, tr->t_in_seg, &x, &v);
        tr->x = x;
        tr->v = v;
        tr->a = seg->a_const;
    }

    if (x_ref) *x_ref = tr->x;
    if (v_ref) *v_ref = tr->v;
    if (a_ref) *a_ref = tr->a;
}

/* ==========================================================================
 * TUONG THICH NGUOC - xem giai thich o trajectory.h. Tat ca deu quy ve
 * trajectory_replan() voi v0=a0=0 (chuyen dong luon xuat phat tu dung yen).
 * ========================================================================== */

void trajectory_start(trajectory_t *tr, float from, float to, float v_max, float a_max)
{
    trajectory_replan(tr, from, 0.0f, 0.0f, to, v_max, a_max);
}

float trajectory_compute_time(float from, float to, float v_max, float a_max)
{
    trajectory_state_t tmp;
    trajectory_replan(&tmp, from, 0.0f, 0.0f, to, v_max, a_max);

    float total = 0.0f;
    for (int i = 0; i < tmp.seg_count; i++) {
        total += tmp.seg[i].duration;
    }
    return total;
}

void trajectory_start_scaled(trajectory_t *tr, float from, float to,
                              float v_max, float a_max, float target_time)
{
    float t0 = trajectory_compute_time(from, to, v_max, a_max);

    if (t0 <= 1e-6f || target_time <= t0) {
        /* Khong co quang duong, hoac target_time khong lon hon tu nhien -
           dung thang v_max/a_max goc, KHONG scale nguoc (tranh ep nhanh
           hon kha nang vat ly da tinh tu datasheet) */
        trajectory_start(tr, from, to, v_max, a_max);
        return;
    }

    float s = target_time / t0;   /* s > 1 - he so keo dai thoi gian */
    trajectory_start(tr, from, to, v_max / s, a_max / (s * s));
}

void trajectory_start_synced3(trajectory_t tr[3], const float from[3],
                               const float to[3], float v_max, float a_max)
{
    float t0[3];
    for (int i = 0; i < 3; i++) {
        t0[i] = trajectory_compute_time(from[i], to[i], v_max, a_max);
    }

    float T = t0[0];
    if (t0[1] > T) T = t0[1];
    if (t0[2] > T) T = t0[2];

    for (int i = 0; i < 3; i++) {
        trajectory_start_scaled(&tr[i], from[i], to[i], v_max, a_max, T);
    }
}

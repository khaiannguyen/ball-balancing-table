/**
 * @file    trajectory.h
 * @brief   Tang 3 - Trajectory generation, acceleration-continuous replan.
 *
 * SUA (ke hoach sua trajectory + actuator + IK cho servo moi, muc 3): thay
 * trajectory_t/trajectory_start() kieu cu bang trajectory_state_t +
 * trajectory_replan()/trajectory_update() - diem sua CỐT LOI: replan()
 * KHONG con reset ve v=0 nua. Truoc day, moi lan Jetson doi setpoint giua
 * luc truc dang chay (binh thuong o Ball:ON, xem control_mode_balance.c),
 * trajectory_start() coi diem xuat phat luon co v=0 -> mat lien tuc van
 * toc thuc te -> giat servo. trajectory_replan() doc dung (x0,v0,a0) HIEN
 * TAI va xay quy dao tiep tuc tu do, co pha BRAKE tuong minh khi can dao
 * huong.
 *
 * Van CHUA jerk-limited (Jmax) - Phase B, xem ke hoach muc 0. a0 duoc
 * nhan vao replan() nhung CHUA dung toi trong Giai doan 1 (giu tham so de
 * khoi phai doi API lan 2 khi lam Phase B).
 *
 * Module thuan C, KHONG phu thuoc HAL/RTOS/Tang 1-2 - de unit-test rieng
 * tren PC (checklist 8 test case, xem ke hoach muc 3.5).
 *
 * TUONG THICH NGUOC: trajectory_t la alias cua trajectory_state_t,
 * trajectory_start()/trajectory_is_done()/trajectory_compute_time()/
 * trajectory_start_scaled()/trajectory_start_synced3() van giu nguyen ten
 * va hanh vi (chuyen dong LUON xuat phat tu dung yen, v0=a0=0) - dung cho
 * Home/Calib (khong lien quan Ball:ON, khong can biet v0/a0 thuc). Ball:ON
 * (control_mode_balance.c) PHAI dung THANG trajectory_replan()/
 * trajectory_update() de tan dung dung diem sua cot loi noi tren.
 */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRAJ_PHASE_BRAKE = 0,   /* dao huong: phanh ve v=0 truoc khi di tiep */
    TRAJ_PHASE_ACCEL,
    TRAJ_PHASE_CRUISE,
    TRAJ_PHASE_DECEL,
} trajectory_phase_t;

/* 1 doan quy dao voi gia toc HANG SO xuyen suot doan (BRAKE/ACCEL/DECEL:
 * a_const = +-a_max; CRUISE: a_const = 0). x_start/v_start la trang thai
 * TAI THOI DIEM BAT DAU doan nay (khong phai tai t=0 toan cuc) - cho phep
 * tinh vi tri bang cong thuc dong x(t) = x_start + v_start*t +
 * 0.5*a_const*t^2 voi t la thoi gian TRONG doan. */
typedef struct {
    trajectory_phase_t type;
    float duration;   /* thoi luong doan nay (giay), > 0 */
    float x_start;
    float v_start;
    float a_const;
} trajectory_segment_t;

/**
 * @brief State day du cho 1 truc dang chay trajectory acceleration-
 *        continuous. Khac han kieu cu: KHONG suy v/a tu t_elapsed moi lan
 *        goi - toan bo cac doan (segment) da duoc tinh san tai
 *        trajectory_replan(), trajectory_update() chi "di qua" chung theo
 *        dt. Nho vay doc dung x/v/a hien tai BAT KY LUC NAO (can thiet de
 *        replan() giua chung khong mat lien tuc van toc/gia toc).
 */
typedef struct {
    trajectory_segment_t seg[4];   /* toi da: BRAKE, ACCEL, CRUISE, DECEL */
    int   seg_count;
    int   seg_index;               /* doan dang chay; == seg_count -> DONE */
    float t_in_seg;                /* thoi gian da troi trong doan hien tai */

    float target;
    float v_max;
    float a_max;

    /* Output cua lan trajectory_update() gan nhat = "trajectory reference
     * state", dung lam (x0,v0,a0) cho lan trajectory_replan() ke tiep.
     * LUU Y: day la vi tri THAM CHIEU (command state) tu tinh ra, KHONG
     * PHAI vi tri that cua servo - he thong hien CHUA co encoder phan hoi,
     * khong co cach nao biet servo that su dang o dau (phai ghi ro dieu
     * nay o moi noi dung state.x lam "vi tri hien tai"). */
    float x, v, a;
} trajectory_state_t;

/**
 * @brief (Re)plan quy dao tu trang thai HIEN TAI (x0,v0,a0) toi target moi,
 *        KHONG reset v ve 0.
 *
 *        3 truong hop (xem ke hoach muc 3.3):
 *        - target == x0 va v0 ~ 0: da toi noi, khong tao doan nao.
 *        - v0 CUNG huong (hoac ~0) voi huong can di: xay truc tiep
 *          ACCEL/CRUISE/DECEL tu (x0,v0), tu dong rut gon "tam giac" neu
 *          quang duong con lai khong du de dat v_max.
 *        - v0 NGUOC huong voi huong can di (dao chieu giua chung): them 1
 *          doan BRAKE tuong minh (gia toc hang so, huong ve dung nghich
 *          voi v0) truoc, dua v ve 0 tai 1 diem trung gian, roi tiep tuc
 *          nhu truong hop tren tu diem do.
 *
 * @param tr      State se bi GHI DE HOAN TOAN (khong can init truoc)
 * @param x0      Vi tri hien tai (thuong = tr_cu->x sau update() gan nhat)
 * @param v0      Van toc hien tai (thuong = tr_cu->v sau update() gan nhat)
 * @param a0      Gia toc hien tai - CHUA dung trong Giai doan 1 (danh cho
 *                Phase B jerk-limited sau nay)
 * @param target  Dich moi
 * @param v_max   Van toc toi da (dvi/s)
 * @param a_max   Gia toc toi da (dvi/s^2)
 */
void trajectory_replan(trajectory_state_t *tr,
                        float x0, float v0, float a0,
                        float target, float v_max, float a_max);

/**
 * @brief Goi moi chu ky dieu khien, tra ve x/v/a tuc thoi qua con tro
 *        output. Co the di qua NHIEU doan trong 1 lan goi neu dt lon hon 1
 *        doan (khong duoc bo sot doan nao). Sau khi het tat ca doan, tra
 *        ve dung (target, 0, 0) moi chu ky sau do (khong ngoai suy tiep,
 *        tranh sai so tich luy float).
 *
 * @param tr     State da duoc trajectory_replan() truoc do
 * @param dt     Thoi gian tu lan goi truoc, don vi GIAY
 * @param x_ref  [out] vi tri tham chieu tuc thoi (bo qua neu NULL)
 * @param v_ref  [out] van toc tham chieu tuc thoi (bo qua neu NULL)
 * @param a_ref  [out] gia toc tham chieu tuc thoi (bo qua neu NULL)
 */
void trajectory_update(trajectory_state_t *tr, float dt,
                        float *x_ref, float *v_ref, float *a_ref);

/**
 * @brief Tien ich: kiem tra quy dao da xong chua.
 */
static inline bool trajectory_state_is_done(const trajectory_state_t *tr)
{
    return tr->seg_index >= tr->seg_count;
}

/* ==========================================================================
 * TUONG THICH NGUOC - danh cho Home/Calib (hoac bat ky noi nao khac trong
 * project dang dung API kieu cu), noi chuyen dong LUON xuat phat tu dung
 * yen (v0=a0=0), khong can biet trang thai van toc/gia toc thuc te. KHONG
 * dung cho Ball:ON - noi do PHAI goi thang trajectory_replan()/
 * trajectory_update() o tren de giu dung diem sua cot loi cua file nay.
 * ========================================================================== */

typedef trajectory_state_t trajectory_t;

/**
 * @brief Wrapper tuong thich nguoc = trajectory_replan(..., v0=0, a0=0, ...).
 */
void trajectory_start(trajectory_t *tr, float from, float to, float v_max, float a_max);

static inline bool trajectory_is_done(const trajectory_t *tr)
{
    return trajectory_state_is_done(tr);
}

/**
 * @brief Tinh THOI GIAN TU NHIEN (tong duration cac doan) NEU chay
 *        trajectory_start() voi dung v_max/a_max nay - ham THUAN, khong
 *        sua doi state nao. Dung de dong bo nhieu truc (xem
 *        trajectory_start_synced3()).
 *
 * @return Thoi gian (giay) - 0.0f neu from==to (khong can di chuyen).
 */
float trajectory_compute_time(float from, float to, float v_max, float a_max);

/**
 * @brief Giong trajectory_start(), nhung KEO DAI thoi gian ve dung
 *        target_time (giay) bang cach tu dong scale v_max/a_max xuong theo
 *        ty le. CHI KEO DAI duoc (khong the rut ngan hon thoi gian tu
 *        nhien) - neu target_time <= thoi gian tu nhien, dung thang
 *        v_max/a_max goc (khong scale nguoc, tranh vuot qua kha nang vat
 *        ly that cua servo).
 */
void trajectory_start_scaled(trajectory_t *tr, float from, float to,
                              float v_max, float a_max, float target_time);

/**
 * @brief Tien ich: dong bo 3 truc (vd S1/S2/S3) cung ve dich cung luc, du
 *        quang duong khac nhau. Thoi gian chung = max(T0_1, T0_2, T0_3).
 *
 * @param tr    Mang 3 phan tu trajectory_t (1 cho moi truc)
 * @param from  Mang 3 vi tri bat dau (vi tri THAT, vd doc tu
 *              servo_actuator_get_local())
 * @param to    Mang 3 vi tri dich
 * @param v_max Van toc toi da CHUNG cho ca 3 truc (truoc khi scale)
 * @param a_max Gia toc toi da CHUNG cho ca 3 truc (truoc khi scale)
 */
void trajectory_start_synced3(trajectory_t tr[3], const float from[3],
                               const float to[3], float v_max, float a_max);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_H */

/**
 * @file    servo_actuator.c
 * @brief   Tang 2 - Actuator layer (implementation).
 * @see     servo_actuator.h
 */
#include "servo_actuator.h"
#include <stddef.h>   /* NULL */

/* system_state.h/.c: actuator_state_t la bien static (g_actuator) ben
 * trong system_state.c, chi duoc expose qua con tro
 * system_state_get_actuator_ptr(). Chu ky that:
 *     void actuator_state_publish(actuator_state_t *s, int32_t s1, int32_t s2, int32_t s3); */
#include "system_state.h"

/* SERVO_SLEW_MAX_US_PER_S / SERVO_ACCEL_MAX_US_PER_S2: dinh nghia trong
 * servo_actuator.h (nguon chan ly duy nhat cho servo 40kg.cm dang dung),
 * KHONG dinh nghia lai o day. */

/* ------------------------------------------------------------------------ */
/* Calib tam thoi - CHUA co Mode 0 Calibration that. deadband_us de 0 vi
 * chua do that; nap lai bang servo_actuator_set_calib() khi co du lieu
 * calib that tu Flash. Khoang 1000-2000us nam trong day an toan cua ca
 * servo cu va servo 40kg.cm moi (1500+-360 la gioi han co khi linkage,
 * xem ke hoach sua servo moi muc 5.2) - khong bat buoc doi ngay. */
typedef struct {
    int32_t neutral;
    int32_t min;
    int32_t max;
    int32_t deadband_us;
} servo_axis_calib_t;

static servo_axis_calib_t s_calib[SERVO_CH_COUNT] = {
    [SERVO_CH_S1] = { .neutral = 1500, .min = 1000, .max = 2000, .deadband_us = 0 },
    [SERVO_CH_S2] = { .neutral = 1500, .min = 1000, .max = 2000, .deadband_us = 0 },
    [SERVO_CH_S3] = { .neutral = 1500, .min = 1000, .max = 2000, .deadband_us = 0 },
};

/* Trang thai noi bo moi truc - tuong ung S1_now/S2_now/S3_now */
typedef struct {
    int32_t pos_us;             /* gia tri da ap dung lan cuoi (S_now that su) */
    int32_t target_us;          /* dich muon toi - dung cho che do vi tri/incremental */
    int32_t last_dir;           /* -1 / 0 / +1 - huong lenh gan nhat, cho deadband kick */
    float   velocity_us_s;      /* van toc DAT (che do speed-control), != 0 khi speed_mode */
    float   vel_current_us_s;   /* van toc THAT DANG AP DUNG (sau gioi han gia toc) - luon
                                    cap nhat moi step(), ca 2 che do vi tri lan speed */
    bool    speed_mode;
} servo_axis_state_t;

static servo_axis_state_t s_axis[SERVO_CH_COUNT];

/* Con tro toi actuator_state_t that trong system_state.c (bien static g_actuator,
 * chi expose qua system_state_get_actuator_ptr()). */
static actuator_state_t *s_actuator_state = NULL;

/* ------------------------------------------------------------------------ */

void servo_actuator_init(void)
{
    for (int ch = 0; ch < SERVO_CH_COUNT; ch++) {
        s_axis[ch].pos_us           = s_calib[ch].neutral;
        s_axis[ch].target_us        = s_calib[ch].neutral;
        s_axis[ch].last_dir         = 0;
        s_axis[ch].velocity_us_s    = 0.0f;
        s_axis[ch].vel_current_us_s = 0.0f;
        s_axis[ch].speed_mode       = false;
    }

    s_actuator_state = system_state_get_actuator_ptr();

    servo_pwm_init();  /* dua PWM ve neutral phan cung, xem servo_pwm.c */
}

void servo_actuator_set_target(servo_ch_t ch, int32_t target_us)
{
    if (ch >= SERVO_CH_COUNT) return;
    s_axis[ch].speed_mode = false;
    s_axis[ch].target_us  = target_us; /* chua clamp - step() se clamp o (f) */
}

void servo_actuator_set_velocity(servo_ch_t ch, float us_per_sec)
{
    if (ch >= SERVO_CH_COUNT) return;

    if (us_per_sec == 0.0f) {
        /* Tat che do speed - dung lai dung tai vi tri hien tai, khong giat */
        s_axis[ch].speed_mode = false;
        s_axis[ch].target_us  = s_axis[ch].pos_us;
    } else {
        s_axis[ch].speed_mode    = true;
        s_axis[ch].velocity_us_s = us_per_sec;
    }
}

void servo_actuator_apply_delta(servo_ch_t ch, int32_t delta_us)
{
    if (ch >= SERVO_CH_COUNT) return;
    s_axis[ch].speed_mode = false;
    s_axis[ch].target_us += delta_us; /* chua clamp - step() se clamp o (f) */
}

void servo_actuator_get_local(int32_t *s1, int32_t *s2, int32_t *s3)
{
    if (s1) *s1 = s_axis[SERVO_CH_S1].pos_us;
    if (s2) *s2 = s_axis[SERVO_CH_S2].pos_us;
    if (s3) *s3 = s_axis[SERVO_CH_S3].pos_us;
}

void servo_actuator_set_calib(servo_ch_t ch, int32_t neutral, int32_t min,
                               int32_t max, int32_t deadband_us)
{
    if (ch >= SERVO_CH_COUNT) return;
    s_calib[ch].neutral     = neutral;
    s_calib[ch].min         = min;
    s_calib[ch].max         = max;
    s_calib[ch].deadband_us = deadband_us;
}

void servo_actuator_step(float dt)
{
    if (dt <= 0.0f) return;   /* dt khong hop le - bo qua chu ky nay, tranh
                                  chia cho 0 khi tinh van toc mong muon */

    for (int ch = 0; ch < SERVO_CH_COUNT; ch++) {
        servo_axis_state_t *a = &s_axis[ch];
        servo_axis_calib_t *c = &s_calib[ch];

        /* (a) Van toc MONG MUON chu ky nay: che do speed -> dung thang gia
         * tri dat; che do vi tri/incremental -> suy tu khoang cach con lai
         * chia dt (van toc can co de toi dung target trong 1 buoc neu
         * khong bi gioi han gi them - se bi cat boi (c)/(d) ngay sau). */
        float desired_v;
        if (a->speed_mode) {
            desired_v = a->velocity_us_s;
        } else {
            desired_v = (float)(a->target_us - a->pos_us) / dt;
        }

        /* (b) Anti-windup: khong cho di tiep neu da cham bien dung huong -
         * tranh tich luy an trong khi da bi clamp */
        if ((a->pos_us >= c->max && desired_v > 0.0f) ||
            (a->pos_us <= c->min && desired_v < 0.0f)) {
            desired_v = 0.0f;
        }

        /* (c) Gioi han VAN TOC (slew-rate): cat desired_v ve toi da
         * SERVO_SLEW_MAX_US_PER_S (servo_actuator.h - nguon chan ly duy
         * nhat, dong bo voi Trajectory Engine Tang 3). */
        if (desired_v >  SERVO_SLEW_MAX_US_PER_S) desired_v =  SERVO_SLEW_MAX_US_PER_S;
        if (desired_v < -SERVO_SLEW_MAX_US_PER_S) desired_v = -SERVO_SLEW_MAX_US_PER_S;

        /* (d) Gioi han GIA TOC: van toc THAT ap dung (vel_current_us_s) chi
         * duoc doi toi da SERVO_ACCEL_MAX_US_PER_S2*dt moi chu ky - day la
         * lop AN TOAN CUOI CUNG chan dong dinh/giat co khi, KHONG phai bo
         * tao profile chuyen dong (vai tro do thuoc ve Trajectory Engine
         * Tang 3) - 2 lop khong thay the nhau (xem ke hoach sua servo moi
         * muc 4/23). */
        float max_dv = SERVO_ACCEL_MAX_US_PER_S2 * dt;
        float dv = desired_v - a->vel_current_us_s;
        if (dv >  max_dv) dv =  max_dv;
        if (dv < -max_dv) dv = -max_dv;
        a->vel_current_us_s += dv;

        int32_t delta = (int32_t)(a->vel_current_us_s * dt);

        /* (e) Deadband kick: doi huong so voi lenh truoc -> cong them dung
         * deadband truoc khi ap (bu backlash co khi thuan tuy - KHONG dung
         * de xu ly dao huong, viec do da thuoc ve Trajectory Engine Tang 3,
         * xem ke hoach sua servo moi muc 12). */
        int32_t dir = (delta > 0) - (delta < 0);
        if (dir != 0 && dir != a->last_dir && c->deadband_us > 0) {
            delta += dir * c->deadband_us;
        }
        if (dir != 0) a->last_dir = dir;

        /* (f) Ap dung + clamp cung theo calib */
        a->pos_us += delta;
        if (a->pos_us < c->min) a->pos_us = c->min;
        if (a->pos_us > c->max) a->pos_us = c->max;

        /* Giu target_us trong dai calib de tranh "no dich" am tham khi
         * incremental cong don lien tuc theo 1 huong ma khong ai clamp target */
        if (!a->speed_mode) {
            if (a->target_us < c->min) a->target_us = c->min;
            if (a->target_us > c->max) a->target_us = c->max;
        }

        /* (g) Ra PWM (Tang 1) */
        servo_pwm_write_us((servo_ch_t)ch, (uint16_t)a->pos_us);
    }

    /* (h) Publish snapshot cho task khac (Task_CAN_TX, Task_Display...) qua
     * seqlock - actuator_state_t da co san trong system_state.c. */
    if (s_actuator_state != NULL) {
        actuator_state_publish(s_actuator_state,
                                s_axis[SERVO_CH_S1].pos_us,
                                s_axis[SERVO_CH_S2].pos_us,
                                s_axis[SERVO_CH_S3].pos_us);
    }
}

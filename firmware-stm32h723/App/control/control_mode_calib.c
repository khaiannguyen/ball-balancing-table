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

/* Measurement timing at the 100 Hz control rate. */
#define CALIB_SETTLE_CYCLES        200
#define CALIB_SAMPLE_COUNT         5
#define CALIB_OFFSET_SAMPLE_COUNT  200

/*
 * 2D calibration grid centered around the current neutral position.
 *
 * Each axis spans [-CALIB_GRID_HALF_US, +CALIB_GRID_HALF_US].
 * The grid covers both positive and negative roll/pitch commands so that
 * the fitted model represents the full operating range.
 *
 * S3 is constrained by:
 *     S3_offset = -(S1_offset + S2_offset)
 *
 * This constraint also determines the maximum required S3 excursion.
 */
#define CALIB_GRID_HALF_US  180.0f
#define CALIB_GRID_POINTS   7

// Static_assert((int)(2.0f * CALIB_GRID_HALF_US) <= CALIB_TILT_MAX_US, ...);

/*
 * Fit quality limits for the quadratic inverse-kinematics model.
 * The limits are expressed in servo microseconds.
 */
#define CALIB_LS_RMS_MAX_US     15.0
#define CALIB_LS_MAXERR_MAX_US  40.0

#define CALIB_TRAJ_V_MAX_US_S   3000.0f
#define CALIB_TRAJ_A_MAX_US_S2  8000.0f

/* Measurement state machine for a single calibration point. */
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

static meas_phase_t s_meas_phase    = MEAS_MOVE_START;
static uint16_t     s_settle_ct     = 0;
static uint8_t      s_sample_ct     = 0;
static float        s_sum_roll      = 0.0f;
static float        s_sum_pitch     = 0.0f;
static float        s_result_roll   = 0.0f;
static float        s_result_pitch  = 0.0f;

static calib_sub_state_t s_sub = CALIB_SUB_OFFSET;

static trajectory_t s_move_traj[3];
static float s_roll_offset;
static float s_pitch_offset;

/*
 * Calibration samples used for the quadratic least-squares fit.
 * R and P are measured IMU angles after offset removal.
 * S1 and S2 are servo offsets relative to neutral.
 */
#define LS_MAX_POINTS  (CALIB_GRID_POINTS * CALIB_GRID_POINTS)

static float    s_ls_R[LS_MAX_POINTS];
static float    s_ls_P[LS_MAX_POINTS];
static float    s_ls_S1[LS_MAX_POINTS];
static float    s_ls_S2[LS_MAX_POINTS];
static uint16_t s_ls_count = 0;

/*
 * Quadratic coefficients for S1, S2, and S3.
 * S3 is derived from S3 = -(S1 + S2), so it does not require
 * an independent regression.
 */
static bool  s_ls_ok = false;
static float s_ik_coef[3][CALIB_IK_NUM_COEF];

/* State machine for the 2D calibration grid. */
typedef enum {
    SWEEP_MOVE_START = 0,
    SWEEP_MOVE_RUN,
    SWEEP_SETTLE,
    SWEEP_SAMPLE,
    SWEEP_RETURN_START,
    SWEEP_RETURN_RUN,
    SWEEP_ALL_DONE
} sweep_phase_t;

static sweep_phase_t s_sweep_phase = SWEEP_MOVE_START;
static uint16_t      s_grid_idx    = 0;

/*
 * Optional neutral override for the current calibration run.
 * When disabled, the neutral values stored in calibration data are used.
 */
static bool    s_neutral_override = false;
static int32_t s_ov_n1 = 0;
static int32_t s_ov_n2 = 0;
static int32_t s_ov_n3 = 0;

void control_mode_calib_set_neutral(
    int16_t s1_neutral,
    int16_t s2_neutral,
    int16_t s3_neutral)
{
    s_ov_n1 = s1_neutral;
    s_ov_n2 = s2_neutral;
    s_ov_n3 = s3_neutral;
    s_neutral_override = true;
}

/*
 * Optional hardcoded neutral values used for calibration.
 * Disable this override after the required values have been stored.
 */
#define CALIB_NEUTRAL_HARDCODE_ENABLE  1
#define CALIB_NEUTRAL_HARDCODE_S1      1632
#define CALIB_NEUTRAL_HARDCODE_S2      1549
#define CALIB_NEUTRAL_HARDCODE_S3      1580

/*
 * Optional hardcoded deadband values used when saving calibration data.
 * When enabled, these values replace the existing Flash values.
 */
#define CALIB_DEADBAND_HARDCODE_ENABLE  1
#define CALIB_DEADBAND_HARDCODE_S1      10
#define CALIB_DEADBAND_HARDCODE_S2      10
#define CALIB_DEADBAND_HARDCODE_S3      10

/*
 * Return the neutral values used by the current calibration run.
 * Keeping this access centralized ensures that OFFSET, SWEEP, and SAVE
 * all use the same neutral reference.
 */
static void get_working_neutral(
    const calibration_data_t *c,
    int32_t *n1,
    int32_t *n2,
    int32_t *n3)
{
    if (s_neutral_override) {
        *n1 = s_ov_n1;
        *n2 = s_ov_n2;
        *n3 = s_ov_n3;
    } else {
        *n1 = c->S1_neutral;
        *n2 = c->S2_neutral;
        *n3 = c->S3_neutral;
    }
}

static void set_guide(const char *msg)
{
    strncpy(g_uiData.guideText, msg, sizeof(g_uiData.guideText) - 1);
    g_uiData.guideText[sizeof(g_uiData.guideText) - 1] = '\0';
}

/*
 * Advance all three synchronized trajectories by one control period
 * and publish the resulting servo targets.
 */
static bool traj_run_step(float dt)
{
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

/*
 * Start a synchronized trajectory from the current actuator positions
 * to the requested servo targets.
 */
static void traj_start_to(int32_t s1, int32_t s2, int32_t s3)
{
    int32_t cs1, cs2, cs3;
    servo_actuator_get_local(&cs1, &cs2, &cs3);

    float from[3] = {
        (float)cs1,
        (float)cs2,
        (float)cs3
    };

    float to[3] = {
        (float)s1,
        (float)s2,
        (float)s3
    };

    trajectory_start_synced3(
        s_move_traj,
        from,
        to,
        CALIB_TRAJ_V_MAX_US_S,
        CALIB_TRAJ_A_MAX_US_S2
    );
}

static void send_evt(state_event_t evt)
{
    osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
}

/*
 * Solve A*x = b using Gaussian elimination with partial pivoting.
 *
 * A and b are modified in place. Partial pivoting selects the largest
 * available pivot in each column to reduce numerical error.
 */
#define NCOEF 6

static bool gauss_solve(
    double A[NCOEF][NCOEF],
    double b[NCOEF],
    double x[NCOEF])
{
    for (int col = 0; col < NCOEF; col++) {
        int piv = col;
        double best = fabs(A[col][col]);

        for (int row = col + 1; row < NCOEF; row++) {
            if (fabs(A[row][col]) > best) {
                best = fabs(A[row][col]);
                piv = row;
            }
        }

        if (best < 1e-9) {
            return false;
        }

        if (piv != col) {
            for (int k = 0; k < NCOEF; k++) {
                double t = A[col][k];
                A[col][k] = A[piv][k];
                A[piv][k] = t;
            }

            double t = b[col];
            b[col] = b[piv];
            b[piv] = t;
        }

        for (int row = col + 1; row < NCOEF; row++) {
            double factor = A[row][col] / A[col][col];

            for (int k = col; k < NCOEF; k++) {
                A[row][k] -= factor * A[col][k];
            }

            b[row] -= factor * b[col];
        }
    }

    for (int row = NCOEF - 1; row >= 0; row--) {
        double s = b[row];

        for (int k = row + 1; k < NCOEF; k++) {
            s -= A[row][k] * x[k];
        }

        x[row] = s / A[row][row];
    }

    return true;
}

/*
 * Build the quadratic feature vector:
 * [R, P, R^2, P^2, R*P, 1].
 */
static void feature_row(double R, double P, double f[NCOEF])
{
    f[0] = R;
    f[1] = P;
    f[2] = R * R;
    f[3] = P * P;
    f[4] = R * P;
    f[5] = 1.0;
}

/*
 * Build the normal equations:
 *     (F^T F)c = F^T Y
 *
 * and solve for the six quadratic model coefficients.
 */
static bool solve_quadratic(
    const float *R,
    const float *P,
    const float *Y,
    uint16_t n,
    double coef_out[NCOEF])
{
    double AtA[NCOEF][NCOEF] = {0};
    double Atb[NCOEF] = {0};

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

/*
 * Compute RMS error, maximum absolute error, and R^2 for a quadratic fit.
 */
static void compute_fit_stats_quad(
    const float *R,
    const float *P,
    const float *Y,
    uint16_t n,
    const double coef[NCOEF],
    double *rms,
    double *max_err,
    double *r2)
{
    double sse = 0.0;
    double sum_y = 0.0;
    double maxe = 0.0;

    for (uint16_t i = 0; i < n; i++) {
        double f[NCOEF];
        feature_row((double)R[i], (double)P[i], f);

        double pred = 0.0;

        for (int k = 0; k < NCOEF; k++) {
            pred += coef[k] * f[k];
        }

        double e = pred - (double)Y[i];

        sse += e * e;

        if (fabs(e) > maxe) {
            maxe = fabs(e);
        }

        sum_y += (double)Y[i];
    }

    double mean_y = sum_y / (double)n;
    double sst = 0.0;

    for (uint16_t i = 0; i < n; i++) {
        double d = (double)Y[i] - mean_y;
        sst += d * d;
    }

    *rms = sqrt(sse / (double)n);
    *max_err = maxe;
    *r2 = (sst > 1e-9) ? (1.0 - sse / sst) : 1.0;
}

/*
 * Fit quadratic models for S1 and S2 from the collected grid samples.
 *
 * S3 is derived from the mechanical constraint:
 *     S3 = -(S1 + S2)
 *
 * The fit is accepted only when both models are solvable and satisfy
 * the configured RMS and maximum-error limits.
 */
static void solve_and_print_ik_coeffs(void)
{
    s_ls_ok = false;

    uint16_t n = s_ls_count;

    if (n < 3 * NCOEF) {
        printf(
            "[calib] LS ERROR: qua it diem (%u), can toi thieu %u de fit bac 2 tin cay\r\n",
            n,
            (unsigned)(3 * NCOEF)
        );
        return;
    }

    double coef_S1[NCOEF];
    double coef_S2[NCOEF];

    bool ok1 = solve_quadratic(
        s_ls_R,
        s_ls_P,
        s_ls_S1,
        n,
        coef_S1
    );

    bool ok2 = solve_quadratic(
        s_ls_R,
        s_ls_P,
        s_ls_S2,
        n,
        coef_S2
    );

    if (!ok1 || !ok2) {
        printf(
            "[calib] LS ERROR: ma tran suy bien - du lieu luoi khong du da dang\r\n"
        );
        return;
    }

    double rms1, max1, r2_1;
    double rms2, max2, r2_2;

    compute_fit_stats_quad(
        s_ls_R,
        s_ls_P,
        s_ls_S1,
        n,
        coef_S1,
        &rms1,
        &max1,
        &r2_1
    );

    compute_fit_stats_quad(
        s_ls_R,
        s_ls_P,
        s_ls_S2,
        n,
        coef_S2,
        &rms2,
        &max2,
        &r2_2
    );

    printf(
        "[calib] Least Squares BAC 2 tu %u diem:\r\n",
        n
    );

    printf(
        "S1 = %.4fR %+.4fP %+.4fR^2 %+.4fP^2 %+.4fRP %+.4f  "
        "(RMS=%.3f max=%.3f R2=%.5f)\r\n",
        coef_S1[0],
        coef_S1[1],
        coef_S1[2],
        coef_S1[3],
        coef_S1[4],
        coef_S1[5],
        rms1,
        max1,
        r2_1
    );

    printf(
        "S2 = %.4fR %+.4fP %+.4fR^2 %+.4fP^2 %+.4fRP %+.4f  "
        "(RMS=%.3f max=%.3f R2=%.5f)\r\n",
        coef_S2[0],
        coef_S2[1],
        coef_S2[2],
        coef_S2[3],
        coef_S2[4],
        coef_S2[5],
        rms2,
        max2,
        r2_2
    );

    bool quality_ok =
        (rms1 <= CALIB_LS_RMS_MAX_US) &&
        (max1 <= CALIB_LS_MAXERR_MAX_US) &&
        (rms2 <= CALIB_LS_RMS_MAX_US) &&
        (max2 <= CALIB_LS_MAXERR_MAX_US);

    if (!quality_ok) {
        printf(
            "[calib] LS CANH BAO: sai so vuot nguong "
            "(RMS_MAX=%.1f MAXERR_MAX=%.1f us) - "
            "TU CHOI ghi Flash, kiem tra co khi/IMU/servo roi CHAY LAI CALIB\r\n",
            CALIB_LS_RMS_MAX_US,
            CALIB_LS_MAXERR_MAX_US
        );
        return;
    }

    /*
     * Apply the S3 constraint directly to the fitted coefficients.
     * This preserves the mechanical relationship for every (R, P).
     */
    for (int k = 0; k < NCOEF; k++) {
        s_ik_coef[0][k] = (float)coef_S1[k];
        s_ik_coef[1][k] = (float)coef_S2[k];
        s_ik_coef[2][k] = (float)(-(coef_S1[k] + coef_S2[k]));
    }

    printf(
        "S3 = suy tu S3=-(S1+S2), da ap len tung he so\r\n"
    );

    s_ls_ok = true;
}

/*
 * Measure IMU attitude at a fixed servo position.
 *
 * The state machine moves to the target, waits for mechanical settling,
 * collects the requested number of samples, then returns to neutral.
 */
static bool run_measurement(
    float dt,
    int32_t s1,
    int32_t s2,
    int32_t s3,
    uint16_t sample_count)
{
    switch (s_meas_phase) {
        case MEAS_MOVE_START:
            traj_start_to(s1, s2, s3);
            s_meas_phase = MEAS_MOVE_RUN;
            break;

        case MEAS_MOVE_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct = 0;
                s_meas_phase = MEAS_SETTLE;
            }
            break;

        case MEAS_SETTLE:
            s_settle_ct++;

            if (s_settle_ct >= CALIB_SETTLE_CYCLES) {
                s_settle_ct = 0;
                s_sample_ct = 0;
                s_sum_roll = 0.0f;
                s_sum_pitch = 0.0f;
                s_meas_phase = MEAS_SAMPLE;
            }
            break;

        case MEAS_SAMPLE: {
            float roll;
            float pitch;
            float vroll;
            float vpitch;

            imu_state_read(
                system_state_get_imu_ptr(),
                &roll,
                &pitch,
                &vroll,
                &vpitch
            );

            s_sum_roll += roll;
            s_sum_pitch += pitch;
            s_sample_ct++;

            if (s_sample_ct >= sample_count) {
                s_result_roll = s_sum_roll / (float)sample_count;
                s_result_pitch = s_sum_pitch / (float)sample_count;
                s_meas_phase = MEAS_RETURN_START;
            }
            break;
        }

        case MEAS_RETURN_START:
            /*
             * Return to the same neutral reference used for the measurement.
             * This is important when a neutral override is active.
             */
            traj_start_to(s1, s2, s3);
            s_meas_phase = MEAS_RETURN_RUN;
            break;

        case MEAS_RETURN_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct = 0;
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

/*
 * Generate the servo targets for one point in the 2D calibration grid.
 *
 * S1 and S2 are swept symmetrically around neutral.
 * S3 is derived from the mechanical constraint:
 *     S3_offset = -(S1_offset + S2_offset)
 *
 * The offsets are calculated from the grid index using floating-point
 * interpolation and rounded independently to avoid accumulated rounding
 * error across the grid.
 */
static void grid_targets(
    uint16_t idx,
    int32_t n1,
    int32_t n2,
    int32_t n3,
    int32_t *s1,
    int32_t *s2,
    int32_t *s3)
{
    int row = idx / CALIB_GRID_POINTS;
    int col = idx % CALIB_GRID_POINTS;

    float s1_off_f =
        -CALIB_GRID_HALF_US +
        (float)row * (2.0f * CALIB_GRID_HALF_US) /
        (float)(CALIB_GRID_POINTS - 1);

    float s2_off_f =
        -CALIB_GRID_HALF_US +
        (float)col * (2.0f * CALIB_GRID_HALF_US) /
        (float)(CALIB_GRID_POINTS - 1);

    int32_t s1_off = (int32_t)lroundf(s1_off_f);
    int32_t s2_off = (int32_t)lroundf(s2_off_f);
    int32_t s3_off = -(s1_off + s2_off);

    *s1 = n1 + s1_off;
    *s2 = n2 + s2_off;
    *s3 = n3 + s3_off;
}

/*
 * Execute the complete 2D calibration sweep.
 *
 * Each grid point is reached through the synchronized trajectory engine,
 * followed by a settling period and IMU averaging. The actuator offsets
 * and corresponding measured angles are stored for the quadratic fit.
 *
 * The system returns to neutral only after the final grid point.
 */
static bool run_sweep_grid(float dt)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    int32_t n1;
    int32_t n2;
    int32_t n3;

    get_working_neutral(c, &n1, &n2, &n3);

    switch (s_sweep_phase) {
        case SWEEP_MOVE_START: {
            int32_t s1;
            int32_t s2;
            int32_t s3;

            grid_targets(
                s_grid_idx,
                n1,
                n2,
                n3,
                &s1,
                &s2,
                &s3
            );

            traj_start_to(s1, s2, s3);
            s_sweep_phase = SWEEP_MOVE_RUN;
            break;
        }

        case SWEEP_MOVE_RUN:
            if (traj_run_step(dt)) {
                s_settle_ct = 0;
                s_sweep_phase = SWEEP_SETTLE;
            }
            break;

        case SWEEP_SETTLE:
            s_settle_ct++;

            if (s_settle_ct >= CALIB_SETTLE_CYCLES) {
                s_settle_ct = 0;
                s_sample_ct = 0;
                s_sum_roll = 0.0f;
                s_sum_pitch = 0.0f;
                s_sweep_phase = SWEEP_SAMPLE;
            }
            break;

        case SWEEP_SAMPLE: {
            float roll;
            float pitch;
            float vroll;
            float vpitch;

            imu_state_read(
                system_state_get_imu_ptr(),
                &roll,
                &pitch,
                &vroll,
                &vpitch
            );

            s_sum_roll += roll;
            s_sum_pitch += pitch;
            s_sample_ct++;

            if (s_sample_ct >= CALIB_SAMPLE_COUNT) {
                s_result_roll =
                    s_sum_roll / (float)CALIB_SAMPLE_COUNT;

                s_result_pitch =
                    s_sum_pitch / (float)CALIB_SAMPLE_COUNT;

                int32_t s1;
                int32_t s2;
                int32_t s3;

                grid_targets(
                    s_grid_idx,
                    n1,
                    n2,
                    n3,
                    &s1,
                    &s2,
                    &s3
                );

                printf(
                    "%ld,%ld,%ld,%.2f,%.2f\r\n",
                    (long)(s1 - n1),
                    (long)(s2 - n2),
                    (long)(s3 - n3),
                    (double)s_result_roll,
                    (double)s_result_pitch
                );

                /*
                 * Remove the neutral IMU offset before storing the sample.
                 * The resulting R/P values are centered around zero and match
                 * the coordinate convention used by the control model.
                 */
                if (s_ls_count < LS_MAX_POINTS) {
                    s_ls_R[s_ls_count] =
                        s_result_roll - s_roll_offset;

                    s_ls_P[s_ls_count] =
                        s_result_pitch - s_pitch_offset;

                    s_ls_S1[s_ls_count] =
                        (float)(s1 - n1);

                    s_ls_S2[s_ls_count] =
                        (float)(s2 - n2);

                    s_ls_count++;
                }

                if (s_grid_idx + 1 < LS_MAX_POINTS) {
                    s_grid_idx++;
                    s_sweep_phase = SWEEP_MOVE_START;
                } else {
                    /*
                     * Return to neutral only after the complete grid has
                     * been measured.
                     */
                    s_sweep_phase = SWEEP_RETURN_START;
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
    s_sub = CALIB_SUB_OFFSET;
    s_meas_phase = MEAS_MOVE_START;

    /*
     * A neutral override is disabled by default and must be explicitly
     * configured by the caller or the compile-time override below.
     */
    s_neutral_override = false;

#if CALIB_NEUTRAL_HARDCODE_ENABLE
    control_mode_calib_set_neutral(
        CALIB_NEUTRAL_HARDCODE_S1,
        CALIB_NEUTRAL_HARDCODE_S2,
        CALIB_NEUTRAL_HARDCODE_S3
    );
#endif

    set_guide("Calib: Get offset IMU...");
}

void control_mode_calib_step(float dt)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    int32_t n1;
    int32_t n2;
    int32_t n3;

    get_working_neutral(c, &n1, &n2, &n3);

    bool done;

    switch (s_sub) {
        case CALIB_SUB_OFFSET:
            done = run_measurement(
                dt,
                n1,
                n2,
                n3,
                CALIB_OFFSET_SAMPLE_COUNT
            );

            if (done) {
                s_roll_offset = s_result_roll;
                s_pitch_offset = s_result_pitch;

                s_sub = CALIB_SUB_SWEEP_GRID;
                s_sweep_phase = SWEEP_MOVE_START;
                s_grid_idx = 0;
                s_ls_count = 0;

                printf(
                    "== SWEEP GRID 2D (%dx%d diem, S3=-(S1+S2)) ==\r\n",
                    CALIB_GRID_POINTS,
                    CALIB_GRID_POINTS
                );

                printf("S1,S2,S3,roll,pitch\r\n");

                set_guide("Calib: Sweep grid 2D...");
            }
            break;

        case CALIB_SUB_SWEEP_GRID:
            done = run_sweep_grid(dt);

            if (done) {
                printf(
                    "== SWEEP DONE (%u diem) - dang giai Least Squares bac 2 ==\r\n",
                    (unsigned)s_ls_count
                );

                solve_and_print_ik_coeffs();

                s_sub = CALIB_SUB_SAVE;

                set_guide("Calib: Write offset to Flash...");
            }
            break;

        case CALIB_SUB_SAVE: {
            printf(
                "[calib] roll_offset=%.2f pitch_offset=%.2f "
                "(trung binh %d mau, ~2s tai neutral)\r\n",
                (double)s_roll_offset,
                (double)s_pitch_offset,
                (int)CALIB_OFFSET_SAMPLE_COUNT
            );

            /*
             * Start from the existing calibration data so fields not
             * recalculated by this calibration remain unchanged.
             */
            calibration_data_t out = *c;

            out.roll_offset = s_roll_offset;
            out.pitch_offset = s_pitch_offset;

#if CALIB_DEADBAND_HARDCODE_ENABLE
            /*
             * Apply the configured deadband values before saving.
             * These values take precedence over the existing Flash data.
             */
            out.deadband_S1 = CALIB_DEADBAND_HARDCODE_S1;
            out.deadband_S2 = CALIB_DEADBAND_HARDCODE_S2;
            out.deadband_S3 = CALIB_DEADBAND_HARDCODE_S3;

            printf(
                "[calib] Ap dung deadband moi: S1=%d S2=%d S3=%d\r\n",
                out.deadband_S1,
                out.deadband_S2,
                out.deadband_S3
            );
#endif

            /*
             * Store the same neutral reference that was used throughout
             * OFFSET and SWEEP. calibration_data_save() derives the
             * corresponding actuator limits from this neutral position.
             */
            if (s_neutral_override) {
                out.S1_neutral = (int16_t)n1;
                out.S2_neutral = (int16_t)n2;
                out.S3_neutral = (int16_t)n3;

                printf(
                    "[calib] Ap dung neutral moi: S1=%ld S2=%ld S3=%ld\r\n",
                    (long)n1,
                    (long)n2,
                    (long)n3
                );
            }

            /*
             * Only replace the stored IK model when the fitted model
             * passed both numerical solvability and quality checks.
             * Otherwise the previously validated coefficients are kept.
             */
            if (s_ls_ok) {
                for (int i = 0; i < 3; i++) {
                    for (int k = 0; k < CALIB_IK_NUM_COEF; k++) {
                        out.ik_coef[i][k] = s_ik_coef[i][k];
                    }
                }

                printf(
                    "[calib] Da ghi 18 he so IK bac 2 moi vao ik_coef[3][6]\r\n"
                );
            } else {
                printf(
                    "[calib] CANH BAO: Least Squares that bai/khong dat nguong "
                    "- GIU NGUYEN he so IK cu\r\n"
                );
            }

            if (calibration_data_save(&out)) {
                printf("[calib] Write FLASH Successfully\r\n");

                servo_actuator_set_calib(
                    SERVO_CH_S1,
                    out.S1_neutral,
                    out.S1_min,
                    out.S1_max,
                    out.deadband_S1
                );

                servo_actuator_set_calib(
                    SERVO_CH_S2,
                    out.S2_neutral,
                    out.S2_min,
                    out.S2_max,
                    out.deadband_S2
                );

                servo_actuator_set_calib(
                    SERVO_CH_S3,
                    out.S3_neutral,
                    out.S3_min,
                    out.S3_max,
                    out.deadband_S3
                );

                s_sub = CALIB_SUB_DONE;

                set_guide("Calibrate done! READY...");

                send_evt(EVT_MODE_CALIB_DONE);
            } else {
                printf("[calib] WRITE FLASH FAIL\r\n");

                s_sub = CALIB_SUB_ERROR;

                set_guide("Calib ERR - WRITE FLASH FAIL");

                send_evt(EVT_MODE_CALIB_FAILED);
            }

            /*
             * The override is valid only for the current calibration run.
             * Clear it before the next run to avoid carrying state forward.
             */
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

#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Persistent calibration data format.
 *
 * The calibration model uses a second-order polynomial for each servo:
 *
 *   S_i = c0*R + c1*P + c2*R^2 + c3*P^2 + c4*R*P + c5
 *
 * A versioned magic value prevents older calibration layouts from being
 * interpreted as the current structure.
 */

#define CALIB_MAGIC     0xC0FFEE03u
#define CALIB_VERSION   3u

/*
 * Maximum mechanical servo offset used to derive the calibration limits.
 *
 * This is the absolute safety limit for servo offsets. It is separate from
 * the smaller calibration sweep range used by control_mode_calib.c.
 */
#define CALIB_TILT_MAX_US   360

/* Number of coefficients in the second-order IK model. */
#define CALIB_IK_NUM_COEF   6

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t _reserved0;

    int16_t S1_neutral;
    int16_t S2_neutral;
    int16_t S3_neutral;

    int16_t S1_max;
    int16_t S2_max;
    int16_t S3_max;

    int16_t S1_min;
    int16_t S2_min;
    int16_t S3_min;

    float roll_offset;
    float pitch_offset;

    /*
     * Second-order IK coefficients for servo i:
     *
     *   S_i = c0*R + c1*P + c2*R^2
     *       + c3*P^2 + c4*R*P + c5
     *
     * R and P are roll/pitch values expressed around the calibrated
     * neutral position.
     */
    float ik_coef[3][CALIB_IK_NUM_COEF];

    /*
     * Servo deadband measured during manual calibration.
     *
     * These values are passed to the actuator layer and are expressed
     * in PWM microseconds.
     */
    int16_t deadband_S1;
    int16_t deadband_S2;
    int16_t deadband_S3;

    uint16_t _reserved1;

    /*
     * CRC32 calculated over the structure excluding this field.
     *
     * The CRC protects the persistent calibration data against corrupted
     * or partially written Flash contents.
     */
    uint32_t crc32;

} calibration_data_t;

/*
 * Load calibration data from Flash into RAM.
 *
 * Returns false when the stored magic, version, or CRC is invalid.
 * In that case, safe default values are loaded instead.
 */
bool calibration_data_load(void);

/*
 * Save calibration data to Flash.
 *
 * The function enforces the current format and derives servo limits from
 * the neutral positions and CALIB_TILT_MAX_US before calculating the CRC.
 *
 * Returns false when the Flash operation or verification fails.
 */
bool calibration_data_save(const calibration_data_t *in);

/*
 * Return a read-only pointer to the active calibration data.
 *
 * Calibration data is only updated while the system is in the calibration
 * state, so normal control-loop readers do not require additional locking.
 */
const calibration_data_t *calibration_data_get_ptr(void);

/* Return true when the active RAM copy passed the Flash integrity checks. */
bool calibration_data_is_valid(void);

/*
 * Initialize safe default calibration values.
 *
 * Zero IK coefficients produce zero servo offsets, keeping all servos at
 * their neutral positions until valid calibration data is available.
 */
void calibration_data_set_defaults(calibration_data_t *out);

/* Apply the active calibration limits and deadbands to the actuator layer. */
void calibration_data_apply_to_actuator(void);

/*
 * Temporarily extend the watchdog timeout around the blocking Flash
 * erase/program operation performed during boot calibration handling.
 */
void calibration_data_iwdg_widen_for_boot(void);

/* Restore the normal watchdog configuration after Flash operations. */
void calibration_data_iwdg_restore_orig(void);

#endif /* CALIBRATION_DATA_H */

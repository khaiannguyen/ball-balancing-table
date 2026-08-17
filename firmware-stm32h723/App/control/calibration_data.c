#include "calibration_data.h"
#include "main.h"          // HAL_CRC, HAL_FLASH, hcrc (handle CubeMX sinh)
#include "servo_actuator.h"
#include <string.h>
#include <stdio.h>

/* CALIB_FLASH_ADDR = 128K cuối cùng của Flash (0x080E0000-0x080FFFFF),
 * KHỚP với vùng FLASH_CALIB đã chừa trong STM32H723ZGTX_FLASH.ld. Struct
 * v3 lớn hơn v2 (18 float ik_coef thay vì 6 float A_i/B_i) nhưng vẫn nhỏ
 * hơn rất nhiều so với 128K - _Static_assert bên dưới tự kiểm tra lại. */
#define CALIB_FLASH_ADDR      0x080E0000UL
#define CALIB_FLASH_SECTOR    FLASH_SECTOR_7
#define CALIB_FLASH_BANK      FLASH_BANK_1
#define CALIB_REGION_SIZE     0x00020000UL   /* 128K - PHẢI khớp FLASH_CALIB trong .ld */

extern CRC_HandleTypeDef hcrc;
extern IWDG_HandleTypeDef hiwdg1;

/* ==========================================================================
 * GIỮ NGUYÊN như v2 - kỹ thuật nới IWDG quanh thao tác Flash blocking. Xem
 * giải thích gốc: IWDG1 Prescaler=32/Reload=500 -> timeout ~0.5s, trong khi
 * erase 1 sector 128K có thể mất tới ~4s (blocking, không refresh được giữa
 * chừng) -> nới lên ~32s ngay trước erase+program, trả về đúng cấu hình gốc
 * ngay sau khi ghi xong.
 * ========================================================================== */
#define IWDG_ORIG_PRESCALER_REG   IWDG_PRESCALER_32
#define IWDG_ORIG_RELOAD          500u
#define IWDG_CALIB_PRESCALER_REG  IWDG_PRESCALER_256
#define IWDG_CALIB_RELOAD         4000u

static void iwdg_set_timeout_raw(uint32_t prescaler_reg, uint32_t reload)
{
    IWDG_TypeDef *inst = hiwdg1.Instance;
    WRITE_REG(inst->KR, 0x5555U);
    WRITE_REG(inst->PR, prescaler_reg);
    WRITE_REG(inst->RLR, reload);
    while (READ_BIT(inst->SR, (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {
        /* chờ phần cứng cập nhật xong PR/RLR (theo RM0468) */
    }
    WRITE_REG(inst->KR, 0xAAAAU);
}

void calibration_data_iwdg_widen_for_boot(void)
{
    iwdg_set_timeout_raw(IWDG_CALIB_PRESCALER_REG, IWDG_CALIB_RELOAD);
}
void calibration_data_iwdg_restore_orig(void)
{
    iwdg_set_timeout_raw(IWDG_ORIG_PRESCALER_REG, IWDG_ORIG_RELOAD);
}

static calibration_data_t s_calib;
static bool                s_valid = false;

static uint32_t calc_crc32(const calibration_data_t *d)
{
    uint32_t len_words = (uint32_t)((sizeof(*d) - sizeof(d->crc32)) / sizeof(uint32_t));
    return HAL_CRC_Calculate(&hcrc, (uint32_t *)d, len_words);
}

void calibration_data_set_defaults(calibration_data_t *out)
{
    memset(out, 0, sizeof(*out));
    out->magic   = CALIB_MAGIC;
    out->version = CALIB_VERSION;

    /* Vị trí home đo thực nghiệm bằng thước nước - mặt bàn đỡ bóng song
     * song mặt bàn đặt servo tại vị trí này. */
    out->S1_neutral = 1586; out->S2_neutral = 1496; out->S3_neutral = 1564;
    out->S1_min = out->S1_neutral - CALIB_TILT_MAX_US;
    out->S2_min = out->S2_neutral - CALIB_TILT_MAX_US;
    out->S3_min = out->S3_neutral - CALIB_TILT_MAX_US;
    out->S1_max = out->S1_neutral + CALIB_TILT_MAX_US;
    out->S2_max = out->S2_neutral + CALIB_TILT_MAX_US;
    out->S3_max = out->S3_neutral + CALIB_TILT_MAX_US;

    out->roll_offset  = 0.0f;
    out->pitch_offset = 0.0f;

    /* ik_coef[*][*] đã = 0 do memset() ở trên -> IK (bậc 2) luôn ra
     * S1=S2=S3=0 nếu chưa calib thật - an toàn, không di chuyển sai hướng
     * (giống hệt nguyên tắc A_i/B_i=0 của v2, chỉ khác số lượng hệ số). */

    out->deadband_S1 = 10;
    out->deadband_S2 = 8;
    out->deadband_S3 = 12;

    out->crc32 = calc_crc32(out);
}

bool calibration_data_load(void)
{
    calibration_data_t tmp;
    memcpy(&tmp, (const void *)CALIB_FLASH_ADDR, sizeof(tmp));

    if (tmp.magic != CALIB_MAGIC || tmp.version != CALIB_VERSION) {
        /* Bao gồm cả trường hợp Flash đang chứa dữ liệu v2 cũ (magic khác
         * do đã đổi 0xC0FFEE02 -> 0xC0FFEE03, layout struct cũng đổi hoàn
         * toàn: A1,B1,A2,B2,A3,B3 kiểu bậc 1 KHÔNG tương thích với
         * ik_coef[3][6] kiểu bậc 2) - coi như chưa calib, bắt buộc chạy lại
         * Mode Calib mới, không cố nạp/diễn giải sai field cũ. */
        calibration_data_set_defaults(&s_calib);
        s_valid = false;
        return false;
    }

    uint32_t crc_check = calc_crc32(&tmp);
    if (crc_check != tmp.crc32) {
        calibration_data_set_defaults(&s_calib);
        s_valid = false;
        return false;
    }

    s_calib = tmp;
    s_valid = true;
    return true;
}

bool calibration_data_save(const calibration_data_t *in)
{
    _Static_assert(sizeof(calibration_data_t) <= CALIB_REGION_SIZE,
                   "calibration_data_t vuot qua vung Flash da chua (CALIB_REGION_SIZE)");

    calibration_data_t tmp = *in;
    tmp.magic    = CALIB_MAGIC;
    tmp.version  = CALIB_VERSION;
    tmp._reserved0 = 0;
    tmp._reserved1 = 0;

    /* Tự tính lại min/max theo đúng ràng buộc cơ khí - KHÔNG dùng giá trị
     * min/max người gọi truyền vào, luôn ép về neutral ± CALIB_TILT_MAX_US. */
    tmp.S1_min = tmp.S1_neutral - CALIB_TILT_MAX_US;
    tmp.S2_min = tmp.S2_neutral - CALIB_TILT_MAX_US;
    tmp.S3_min = tmp.S3_neutral - CALIB_TILT_MAX_US;
    tmp.S1_max = tmp.S1_neutral + CALIB_TILT_MAX_US;
    tmp.S2_max = tmp.S2_neutral + CALIB_TILT_MAX_US;
    tmp.S3_max = tmp.S3_neutral + CALIB_TILT_MAX_US;

    tmp.crc32 = calc_crc32(&tmp);

    uint8_t buf[((sizeof(tmp) + 31u) / 32u) * 32u];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &tmp, sizeof(tmp));

    HAL_StatusTypeDef st;
    st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return false;

    iwdg_set_timeout_raw(IWDG_CALIB_PRESCALER_REG, IWDG_CALIB_RELOAD);

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = CALIB_FLASH_BANK;
    erase.Sector       = CALIB_FLASH_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;

    st = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (st != HAL_OK) {
        iwdg_set_timeout_raw(IWDG_ORIG_PRESCALER_REG, IWDG_ORIG_RELOAD);
        HAL_FLASH_Lock();
        return false;
    }

    for (uint32_t off = 0; off < sizeof(buf); off += 32u) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                CALIB_FLASH_ADDR + off,
                                (uint32_t)(uintptr_t)&buf[off]);
        if (st != HAL_OK) {
            iwdg_set_timeout_raw(IWDG_ORIG_PRESCALER_REG, IWDG_ORIG_RELOAD);
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();

    iwdg_set_timeout_raw(IWDG_ORIG_PRESCALER_REG, IWDG_ORIG_RELOAD);

    calibration_data_t verify;
    memcpy(&verify, (const void *)CALIB_FLASH_ADDR, sizeof(verify));
    if (memcmp(&verify, &tmp, sizeof(tmp)) != 0) {
        s_valid = false;
        return false;
    }

    s_calib = tmp;
    s_valid = true;
    return true;
}

const calibration_data_t *calibration_data_get_ptr(void) { return &s_calib; }
bool calibration_data_is_valid(void) { return s_valid; }

void calibration_data_apply_to_actuator(void)
{
    servo_actuator_set_calib(SERVO_CH_S1, s_calib.S1_neutral, s_calib.S1_min,
                              s_calib.S1_max, s_calib.deadband_S1);
    servo_actuator_set_calib(SERVO_CH_S2, s_calib.S2_neutral, s_calib.S2_min,
                              s_calib.S2_max, s_calib.deadband_S2);
    servo_actuator_set_calib(SERVO_CH_S3, s_calib.S3_neutral, s_calib.S3_min,
                              s_calib.S3_max, s_calib.deadband_S3);
    printf("deadband done (S1=%d S2=%d S3=%d)\r\n",
           s_calib.deadband_S1, s_calib.deadband_S2, s_calib.deadband_S3);
}

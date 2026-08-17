#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void imu_debug_dump_regs(void);   // debug tạm, xoá sau khi ổn định dây

#define MPU6500_WHO_AM_I_REG   0x75
#define MPU6500_WHO_AM_I_VAL   0x70   // giá trị datasheet MPU6500 (kiểm tra lại đúng chip bạn có, MPU6500 thường 0x70, một số biến thể 0x71)

#define MPU6500_PWR_MGMT_1     0x6B
#define MPU6500_ACCEL_XOUT_H   0x3B
#define MPU6500_GYRO_XOUT_H    0x43
#define MPU6500_SMPLRT_DIV     0x19   // dùng làm thanh ghi "scratch" test ghi/đọc SPI, an toàn để thử nghiệm

#define MPU6500_CONFIG         0x1A   // DLPF_CFG nằm ở bit[2:0] thanh ghi này
#define MPU6500_GYRO_CONFIG     0x1B
#define MPU6500_ACCEL_CONFIG    0x1C

#define MPU6500_INT_PIN_CFG    0x37
#define MPU6500_INT_ENABLE     0x38
#define MPU6500_INT_STATUS     0x3A

#define MPU6500_H_RESET_BIT     0x80   // bit7 của PWR_MGMT_1

/* ---- FIX QUAN TRỌNG ----
 * Theo datasheet MPU-6500 register map, thanh ghi INT_ENABLE (0x38):
 *   bit4 = WOM_EN
 *   bit3 = FIFO_OFLOW_EN
 *   bit0 = RAW_RDY_EN   <-- bit bật ngắt "Data Ready", đây mới là bit cần cho xung 1kHz
 * Code cũ ghi 0x02 (bit1) — bit1 là reserved, không làm gì cả, nên INT Data Ready
 * không bao giờ được bật dù ghi "thành công".
 */
#define MPU6500_INT_ENABLE_RAW_RDY   0x01

/* ---- FIX QUAN TRỌNG #2: bật DLPF để SMPLRT_DIV có tác dụng ----
 * Sau H_RESET, CONFIG (0x1A) mặc định = 0x00 (DLPF_CFG=0 = bypass filter).
 * Ở chế độ bypass, accelerometer bị giới hạn CỨNG ở 4kHz (không đổi được bằng
 * SMPLRT_DIV), và cờ RAW_DATA_RDY chỉ set khi accel+gyro đều sẵn sàng -> bị
 * kẹt ở tốc độ của đường chậm hơn (accel) = 4kHz. Đây là lý do sample rate
 * đo được đúng ~4000Hz dù không ai chủ động set giá trị đó.
 * DLPF_CFG = 3 đưa internal sample rate về 1kHz cho cả accel & gyro (băng
 * thông ~41-44Hz, hợp lý cho control loop cân bằng), sau đó SMPLRT_DIV mới
 * thật sự có tác dụng chia tần số như ý.
 */
#define MPU6500_DLPF_CFG_1KHZ   0x03

bool imu_mpu6500_init(SPI_HandleTypeDef *hspi);
bool imu_mpu6500_who_am_i_check(void);
void imu_mpu6500_read_raw(int16_t *accel, int16_t *gyro); // polling, blocking
void imu_read_dma_start(void);


uint32_t imu_get_and_reset_sample_count(void); // debug tạm, đo tần số DMA/EXTI — xoá sau khi đo xong

// debug tạm: in số lần EXTI gọi tới imu_read_dma_start(), số lần bị skip vì SPI
// bận, và số lần HAL_SPI_TransmitReceive_DMA thất bại — giúp xác định vì sao
// sample rate = 0Hz đang xảy ra ở khâu nào (EXTI không tới, hay DMA start lỗi).
void imu_debug_print_dma_counters(void);

#endif

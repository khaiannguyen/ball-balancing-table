#include "imu_mpu6500.h"
#include <stdio.h>
#include "main.h"   // IMU_CS_Pin / IMU_CS_GPIO_Port, IMU_INT_Pin, IMU_INT_EXTI_IRQn — CubeMX define sẵn ở đây

#include "system_state.h"      // imu_raw_state_write()
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

static SPI_HandleTypeDef *s_hspi;

#define IMU_DMA_BUF_LEN 15  // 1 byte lệnh + 14 byte data (accel 6 + temp 2 + gyro 6)

#define IMU_DMA_BUF_LEN_ALIGNED 32   // căn chỉnh cache line, dùng cho khai báo mảng
__attribute__((aligned(32)))
static uint8_t s_dma_tx_buf[IMU_DMA_BUF_LEN_ALIGNED];
__attribute__((aligned(32)))
static uint8_t s_dma_rx_buf[IMU_DMA_BUF_LEN_ALIGNED];

static inline void imu_cs_low(void)  { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); }
static inline void imu_cs_high(void) { HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); }
static volatile uint32_t s_imu_sample_count = 0;

extern osThreadId_t ImuFusionTaskHandle;   // handle task, khai báo extern từ freertos.c

static volatile uint32_t s_dma_start_attempts = 0;   // debug: số lần EXTI gọi tới hàm này
static volatile uint32_t s_dma_start_busy_skip = 0;  // debug: số lần bị skip vì SPI đang bận
static volatile uint32_t s_dma_start_fail = 0;       // debug: số lần HAL_SPI_TransmitReceive_DMA thất bại
static volatile uint32_t s_dma_error_count = 0;      // debug: số lần HAL_SPI_ErrorCallback bị gọi
static volatile uint32_t s_last_spi_error_code = 0;  // debug: mã lỗi SPI gần nhất (xem HAL_SPI_GetError)

void imu_read_dma_start(void)
{
    s_dma_start_attempts++;

    // Guard: nếu transfer DMA trước chưa xong (SPI đang bận) thì bỏ qua lần gọi này,
    // tránh chồng lệnh HAL_SPI_TransmitReceive_DMA lên 1 transfer đang chạy.
    // Đặt guard ở đây (thay vì ở nơi gọi) để bất kỳ ai gọi hàm này — kể cả
    // exti_dispatch.c — đều không cần biết gì về SPI state.
    if (HAL_SPI_GetState(s_hspi) != HAL_SPI_STATE_READY) {
        s_dma_start_busy_skip++;
        return;
    }

    s_dma_tx_buf[0] = MPU6500_ACCEL_XOUT_H | 0x80;

    SCB_CleanDCache_by_Addr((uint32_t*)s_dma_tx_buf, IMU_DMA_BUF_LEN_ALIGNED);

    // các byte còn lại trong tx buffer để 0x00 (dummy) — chỉ cần set 1 lần lúc đầu chương trình,
    // không cần memset mỗi lần gọi vì giá trị dummy không đổi
    //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(s_hspi, s_dma_tx_buf, s_dma_rx_buf, IMU_DMA_BUF_LEN);
    if (st != HAL_OK) {
        s_dma_start_fail++;
        imu_cs_high();   // DMA không thật sự chạy -> tự thả CS ra, tránh giữ chân CS thấp mãi
        printf("DMA start FAIL: st=%d hal_error=%lu\r\n", st, HAL_SPI_GetError(s_hspi));
    }
}

/* Nếu HAL_SPI_TransmitReceive_DMA "thành công" (state chuyển BUSY) nhưng sau đó
 * không bao giờ tới HAL_SPI_RxCpltCallback, rất có thể HAL đang tự bắt 1 lỗi nội
 * bộ (Overrun/ModeFault/CRC/Framing...) rồi gọi callback lỗi này để reset state
 * về READY — mà weak-default của HAL_SPI_ErrorCallback không làm gì cả nên lỗi
 * bị nuốt âm thầm, đúng như số liệu "busy_skip = attempts/2" bạn đang thấy.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        s_dma_error_count++;
        s_last_spi_error_code = HAL_SPI_GetError(hspi);
        imu_cs_high();   // đảm bảo nhả CS nếu lỗi xảy ra giữa chừng transfer
    }
}

void imu_debug_print_dma_counters(void)
{
    printf("DMA attempts=%lu busy_skip=%lu fail=%lu spi_error=%lu last_err_code=0x%08lX\r\n",
           s_dma_start_attempts, s_dma_start_busy_skip, s_dma_start_fail,
           s_dma_error_count, s_last_spi_error_code);
}

static uint8_t imu_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = {0};

    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, HAL_MAX_DELAY);
    imu_cs_high();

    if (st != HAL_OK) {
        printf("SPI read err reg 0x%02X: st=%d, hal_error=%lu\r\n", reg, st, HAL_SPI_GetError(s_hspi));
    }

    return rx[1];
}

static bool imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };
    uint8_t rx[2] = {0};

    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(s_hspi, tx, rx, 2, HAL_MAX_DELAY);
    imu_cs_high();

    if (st != HAL_OK) {
        printf("SPI write err reg 0x%02X: st=%d, hal_error=%lu\r\n", reg, st, HAL_SPI_GetError(s_hspi));
        return false;
    }
    return true;
}

/* Ghi rồi đọc lại ngay để xác nhận thanh ghi thật sự đổi giá trị.
 * Đây là công cụ chẩn đoán chính: nếu readback != value, nghĩa là lệnh ghi
 * SPI không có tác dụng lên chip (khác với việc chỉ sai bit logic).
 */
static bool imu_write_reg_verify(uint8_t reg, uint8_t value)
{
    if (!imu_write_reg(reg, value)) {
        return false;
    }
    HAL_Delay(1);
    uint8_t rb = imu_read_reg(reg);
    if (rb != value) {
        printf("IMU write-verify FAIL: reg 0x%02X wrote 0x%02X, read back 0x%02X\r\n",
               reg, value, rb);
        return false;
    }
    printf("IMU write-verify OK: reg 0x%02X = 0x%02X\r\n", reg, value);
    return true;
}

bool imu_mpu6500_init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;

    /* ---- FIX QUAN TRỌNG: disable IRQ NGAY DÒNG ĐẦU TIÊN ----
     * IMU_INT_EXTI_IRQn được CubeMX enable sẵn từ trong MX_GPIO_Init(), tức là
     * TRƯỚC KHI task này chạy. Nếu chip MPU chưa bị mất nguồn thật sự giữa 2 lần
     * nạp code (chỉ MCU bị reset qua debugger) thì nó có thể đã đang phát xung
     * INT từ cấu hình cũ. Khi đó exti_dispatch.c sẽ gọi imu_read_dma_start()
     * ngay giữa lúc hàm init() này đang làm các giao dịch SPI blocking trên
     * CÙNG hspi handle -> 2 giao dịch tranh chấp CS/MISO cùng lúc -> dữ liệu
     * rác + HAL_BUSY. Đó chính xác là lỗi log bạn vừa gửi.
     * Phải disable IRQ trước MỌI thao tác SPI, không phải sau reset/who-am-i.
     */
    HAL_NVIC_DisableIRQ(IMU_INT_EXTI_IRQn);

    imu_cs_high();
    HAL_Delay(50);

    // ---- Software reset toàn bộ thanh ghi về mặc định ----
    imu_write_reg(MPU6500_PWR_MGMT_1, MPU6500_H_RESET_BIT);  // H_RESET=1
    HAL_Delay(100);   // datasheet yêu cầu chờ >=100ms sau reset

    // Sau reset, chip tự về sleep mode (bit SLEEP=1 mặc định) -> đánh thức lại
    imu_write_reg(MPU6500_PWR_MGMT_1, 0x00);
    HAL_Delay(10);

    if (!imu_mpu6500_who_am_i_check()) {
        HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);   // trả lại trạng thái cũ trước khi thoát lỗi
        return false;
    }

    /* Đã xác nhận đường ghi SPI hoạt động đúng (write-verify OK ở log trước),
     * nên bỏ bước test ghi SMPLRT_DIV. LƯU Ý: SMPLRT_DIV (0x19) KHÔNG phải
     * thanh ghi "trung tính" như mình nói trước đó — nó là Sample Rate Divider
     * thật: Sample Rate = 1kHz / (1 + SMPLRT_DIV). Ghi 0x07 vào đó đã âm thầm
     * đổi tốc độ lấy mẫu xuống 125Hz (1000/(1+7)), không phải lỗi nhưng là
     * tác dụng phụ ngoài ý muốn — mình xin lỗi vì chọn nhầm thanh ghi để test.
     */
/*
    bool ok1 = imu_write_reg_verify(MPU6500_INT_PIN_CFG, 0x00);
    // FIX: dùng đúng bit RAW_RDY_EN (bit0 = 0x01), không phải 0x02 (bit1, reserved)
    bool ok2 = imu_write_reg_verify(MPU6500_INT_ENABLE, MPU6500_INT_ENABLE_RAW_RDY);

    // FIX #2: bật DLPF (nếu không, sample rate bị kẹt cứng ~4kHz theo giới hạn
    // phần cứng của accel khi bypass filter, xem giải thích ở header).
    // SMPLRT_DIV=0x00 nghĩa là Sample Rate = 1kHz/(1+0) = 1kHz đúng ý muốn.
    bool ok3 = imu_write_reg_verify(MPU6500_CONFIG, MPU6500_DLPF_CFG_1KHZ);
    bool ok4 = imu_write_reg_verify(MPU6500_SMPLRT_DIV, 0x00);
*/
    bool ok1 = imu_write_reg_verify(MPU6500_INT_PIN_CFG, 0x00);

    bool ok2 = imu_write_reg_verify(MPU6500_INT_ENABLE,
                                    MPU6500_INT_ENABLE_RAW_RDY);

    /* Gyro Full Scale = ±500 dps */
    bool ok3 = imu_write_reg_verify(MPU6500_GYRO_CONFIG, 0x08);

    /* Accel Full Scale = ±4 g */
    bool ok4 = imu_write_reg_verify(MPU6500_ACCEL_CONFIG, 0x08);

    /* DLPF */
    bool ok5 = imu_write_reg_verify(MPU6500_CONFIG,
                                    MPU6500_DLPF_CFG_1KHZ);

    /* 1 kHz */
    bool ok6 = imu_write_reg_verify(MPU6500_SMPLRT_DIV, 0x00);

    uint8_t who_recheck = imu_read_reg(MPU6500_WHO_AM_I_REG);
    printf("INT=%d EN=%d GYRO=%d ACCEL=%d CONFIG=%d SMPLRT=%d WHO=0x%02X\r\n",
           ok1, ok2, ok3, ok4, ok5, ok6, who_recheck);
    printf("PWR_MGMT_2 (0x6C) = 0x%02X\r\n", imu_read_reg(0x6C));

    /* Xoá cờ ngắt EXTI đang "pending" (nếu MPU đã kịp toggle INT trong lúc IRQ
     * bị disable ở trên) trước khi bật IRQ lại, để tránh 1 lần trigger giả
     * ngay lập tức khi vừa enable, dùng dữ liệu SPI vẫn chưa sẵn sàng ổn định. */
    __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT_Pin);
    HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);

    return ok1 && ok2 && ok3 && ok4 && ok5 && ok6;
}

bool imu_mpu6500_who_am_i_check(void)
{
    uint8_t who = imu_read_reg(MPU6500_WHO_AM_I_REG);
    return (who == MPU6500_WHO_AM_I_VAL);
}

void imu_mpu6500_read_raw(int16_t *accel, int16_t *gyro)
{
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};
    tx[0] = MPU6500_ACCEL_XOUT_H | 0x80;

    imu_cs_low();
    HAL_SPI_TransmitReceive(s_hspi, tx, rx, 15, HAL_MAX_DELAY);
    imu_cs_high();

    accel[0] = (int16_t)((rx[1] << 8) | rx[2]);
    accel[1] = (int16_t)((rx[3] << 8) | rx[4]);
    accel[2] = (int16_t)((rx[5] << 8) | rx[6]);
    gyro[0]  = (int16_t)((rx[9]  << 8) | rx[10]);
    gyro[1]  = (int16_t)((rx[11] << 8) | rx[12]);
    gyro[2]  = (int16_t)((rx[13] << 8) | rx[14]);
}

/* FIX QUAN TRỌNG: HAL_SPI_TransmitReceive_DMA() (full-duplex TX+RX) khi hoàn tất
 * gọi HAL_SPI_TxRxCpltCallback(), KHÔNG PHẢI HAL_SPI_RxCpltCallback() (hàm đó chỉ
 * dành cho HAL_SPI_Receive_DMA() - RX-only). Đây là lý do suốt thời gian qua
 * transfer DMA vẫn chạy đúng, không lỗi, nhưng sample_count luôn = 0: callback
 * cũ không bao giờ được HAL gọi tới. */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    if (hspi->Instance == SPI2)
    {
        imu_cs_high();

        // Bắt buộc: xoá cache cho vùng buffer này trước khi đọc, vì DMA ghi
        // thẳng vào RAM vật lý, không thông qua cache -> CPU có thể đang giữ
        // bản cache cũ của vùng nhớ này nếu không invalidate.
        SCB_InvalidateDCache_by_Addr((uint32_t*)s_dma_rx_buf, IMU_DMA_BUF_LEN_ALIGNED);
        int16_t accel[3], gyro[3];
        accel[0] = (int16_t)((s_dma_rx_buf[1]  << 8) | s_dma_rx_buf[2]);
        accel[1] = (int16_t)((s_dma_rx_buf[3]  << 8) | s_dma_rx_buf[4]);
        accel[2] = (int16_t)((s_dma_rx_buf[5]  << 8) | s_dma_rx_buf[6]);
        gyro[0]  = (int16_t)((s_dma_rx_buf[9]  << 8) | s_dma_rx_buf[10]);
        gyro[1]  = (int16_t)((s_dma_rx_buf[11] << 8) | s_dma_rx_buf[12]);
        gyro[2]  = (int16_t)((s_dma_rx_buf[13] << 8) | s_dma_rx_buf[14]);
        //printf("ACC_RAW: X=%d Y=%d Z=%d | GYRO_RAW: X=%d Y=%d Z=%d\r\n", (int)accel[0], (int)accel[1], (int)accel[2], (int)gyro[0],  (int)gyro[1],  (int)gyro[2]);
        imu_raw_state_write(system_state_get_imu_raw_ptr(), accel, gyro);
        s_imu_sample_count++;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR((TaskHandle_t)ImuFusionTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* KHÔNG định nghĩa HAL_GPIO_EXTI_Callback ở đây.
 * Project này đã có 1 dispatcher tập trung duy nhất tại App/bsp/exti_dispatch.c,
 * dispatcher đó gọi thẳng imu_read_dma_start() khi IMU_INT_Pin có sườn lên
 * (guard SPI-busy đã nằm sẵn bên trong imu_read_dma_start() ở trên).
 * Nếu file này cũng định nghĩa HAL_GPIO_EXTI_Callback sẽ bị lỗi linker
 * "multiple definition" — đúng như lỗi bạn gặp.
 */

uint32_t imu_get_and_reset_sample_count(void)
{
    uint32_t count = s_imu_sample_count;
    s_imu_sample_count = 0;
    return count;
}

void imu_debug_dump_regs(void)
{
    HAL_NVIC_DisableIRQ(IMU_INT_EXTI_IRQn);   // chặn EXTI MỚI

    /* Disable EXTI không dừng được 1 transfer DMA ĐÃ lỡ bắt đầu trước đó
     * (hoàn tất DMA là IRQ khác, không bị disable ở trên). Nếu không đợi,
     * lệnh polling đầu tiên bên dưới có thể đụng độ với transfer đang dở
     * dang -> dữ liệu rác (đúng như log WHO_AM_I ra giá trị ngẫu nhiên). */
    uint32_t wait_start = HAL_GetTick();
    while (HAL_SPI_GetState(s_hspi) != HAL_SPI_STATE_READY) {
        if (HAL_GetTick() - wait_start > 5) break;   // timeout an toàn, tránh treo
    }

    printf("--- IMU reg dump ---\r\n");
    printf("WHO_AM_I     (0x75) = 0x%02X\r\n", imu_read_reg(MPU6500_WHO_AM_I_REG));
    printf("PWR_MGMT_1   (0x6B) = 0x%02X\r\n", imu_read_reg(MPU6500_PWR_MGMT_1));
    printf("INT_PIN_CFG  (0x37) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_PIN_CFG));
    printf("INT_ENABLE   (0x38) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_ENABLE));
    printf("INT_STATUS   (0x3A) = 0x%02X\r\n", imu_read_reg(MPU6500_INT_STATUS));
    printf("--------------------\r\n");

    HAL_NVIC_EnableIRQ(IMU_INT_EXTI_IRQn);
}

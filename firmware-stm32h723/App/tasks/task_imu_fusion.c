#include "task_imu_fusion.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include "main.h"

#include "system_state.h"
#include "imu_fusion.h"
#include "imu_mpu6500.h"

//osThreadId_t ImuFusionTaskHandle = NULL;   /* định nghĩa thật, gán ở đầu task */
extern osThreadId_t ImuFusionTaskHandle;
/* =====================================================================
 * KIỂM TRA LẠI VỚI imu_mpu6500.c CỦA BẠN TRƯỚC KHI TIN 2 GIÁ TRỊ NÀY!
 * Tại thời điểm viết file này, imu_mpu6500_init() CHƯA ghi GYRO_CONFIG
 * (0x1B)/ACCEL_CONFIG (0x1C) -> chip đang chạy ở giá trị MẶC ĐỊNH sau
 * H_RESET: Gyro ±250dps (LSB=131.0), Accel ±2g (LSB=16384.0).
 *
 * Nếu bạn đã thêm cấu hình FS tường minh (khuyến nghị ±500dps/±4g) thì SỬA
 * lại đúng 2 số dưới đây cho khớp - sai ở đây không gây lỗi biên dịch, chỉ
 * khiến roll/pitch/vroll/vpitch SAI THEO TỈ LỆ CỐ ĐỊNH mà không có gì báo.
 *
 * Bảng tra LSB/unit chuẩn của MPU6500:
 *   Gyro  ±250 dps  -> 131.0   ±500 dps -> 65.5   ±1000 dps -> 32.8   ±2000 dps -> 16.4
 *   Accel ±2g       -> 16384.0 ±4g      -> 8192.0 ±8g       -> 4096.0 ±16g      -> 2048.0
 * ===================================================================== */
#define GYRO_LSB_PER_DPS     65.5f    /* KHỚP MẶC ĐỊNH ±250dps hiện tại của imu_mpu6500.c - đổi nếu bạn thêm GYRO_CONFIG */
#define ACCEL_LSB_PER_G      8192.0f   /* KHỚP MẶC ĐỊNH ±2g hiện tại của imu_mpu6500.c - đổi nếu bạn thêm ACCEL_CONFIG */

/* Chu kỳ lấy mẫu THẬT của MPU6500 (do ODR cấu hình ở B.1 quyết định, KHÔNG
 * phải chu kỳ FreeRTOS tick) - SỬA cho khớp đúng ODR bạn đã cấu hình.
 * Dùng hằng số cố định thay vì đo bằng tick FreeRTOS (chỉ phân giải 1ms,
 * không đủ chính xác cho dt ở tần số cao) - vì task được đánh thức bởi
 * chính nhịp lấy mẫu phần cứng nên dt gần như cố định tuyệt đối. */
#define IMU_SAMPLE_DT_S      0.001f    /* 1kHz - SỬA nếu B.1 cấu hình ODR khác */

/* Ngưỡng gia tốc để TIN accelerometer (mục đích: bỏ qua measurement update
 * lúc bàn đang rung/dao động mạnh do servo, gia tốc tổng khác xa 1g lúc đó -
 * dùng accel lúc này sẽ làm hỏng ước lượng góc). Đơn vị: g. */
#define ACCEL_TRUST_MIN_G    0.8f
#define ACCEL_TRUST_MAX_G    1.2f

/* Timeout an toàn chờ notify từ ISR - KHÔNG chờ vô hạn, để task vẫn có cơ
 * hội chạy đều (dù dữ liệu cũ) phòng trường hợp ISR/DMA bị lỗi, tránh treo
 * hẳn - lúc đó Task_Watchdog vẫn cần thấy task này "sống" để không reset
 * MCU giữa chừng vì lý do không liên quan (IMU tạm mất tín hiệu vẫn nên để
 * hệ thống chạy, xử lý an toàn nằm ở Task_ControlLoop/failsafe, không phải
 * ở đây). */
#define IMU_NOTIFY_TIMEOUT_MS   20

void StartTaskImuFusion(void *argument)
{
    (void)argument;
    ImuFusionTaskHandle = (osThreadId_t)xTaskGetCurrentTaskHandle();

    /* Q_angle: càng nhỏ càng tin gyro (mượt nhưng trôi chậm nếu bias chưa hội tụ)
     * Q_bias:  càng nhỏ càng tin bias ổn định (bias hội tụ chậm hơn nhưng ít nhiễu)
     * R_measure: càng lớn càng ít tin accel (đỡ nhiễu rung nhưng phản ứng chậm
     *            với thay đổi góc thật) - đây là 3 hằng số CẦN TUNE THỰC TẾ
     *            bằng cách nghiêng bàn tay và so sánh với góc đo bằng thước đo
     *            góc hoặc điện thoại (có cảm biến góc) - giá trị dưới đây chỉ
     *            là điểm khởi đầu hợp lý, không phải giá trị tối ưu cuối cùng. */
    kalman1d_t kf_roll, kf_pitch;
    kalman1d_init(&kf_roll,  0.0008f, 0.003f, 0.0156f);
    kalman1d_init(&kf_pitch, 0.0008f, 0.003f, 0.0148f);

    for (;;)
    {
        /* Chờ ISR báo "có dữ liệu IMU mới" - KHÔNG polling, đúng nguyên tắc
         * mục 3.4 (Task Notification thay vì semaphore, nhẹ hơn ~45%). */

    	uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_NOTIFY_TIMEOUT_MS));

    	int16_t accel_raw[3], gyro_raw[3];
    	imu_raw_state_read(system_state_get_imu_raw_ptr(), accel_raw, gyro_raw);

        /* Dù got==0 (timeout, không có notify mới) vẫn CHẠY TIẾP với dữ liệu
         * cũ trong imu_raw_state_t - để Kalman filter không "đứng hình" và
         * Task_Watchdog vẫn thấy task này sống. Dữ liệu cũ trong vài chu kỳ
         * không gây nguy hiểm tức thời (khác hẳn mất heartbeat CAN, mục 4.4,
         * vốn có thể dẫn tới lệnh sai từ xa - đây chỉ là dữ liệu cảm biến nội
         * bộ, ControlLoop vẫn dùng được tạm trong lúc chờ IMU phục hồi). */
        (void)got;

        /* ---- Đổi đơn vị: raw -> deg/s (gyro), g (accel) ---- */
        float gx = gyro_raw[0] / GYRO_LSB_PER_DPS;
        float gy = gyro_raw[1] / GYRO_LSB_PER_DPS;
        /* gz không dùng - bàn không cần yaw, đúng quyết định thiết kế mục 3.2 */

        float ax = accel_raw[0] / ACCEL_LSB_PER_G;
        float ay = accel_raw[1] / ACCEL_LSB_PER_G;
        float az = -accel_raw[2] / ACCEL_LSB_PER_G;
        //printf("GX:%7.2f dps  GY:%7.2f dps  AX:%6.3f g  AY:%6.3f g  AZ:%6.3f g\r\n",gx, gy, ax, ay, az);

        /* ---- Góc tính từ accelerometer (chỉ đáng tin khi bàn gần như tĩnh) ----
         * QUY ƯỚC TRỤC: roll quay quanh trục X, pitch quay quanh trục Y, Z
         * hướng lên khi bàn nằm ngang. ĐÂY LÀ GIẢ ĐỊNH - PHẢI TỰ KIỂM TRA lại
         * bằng cách nghiêng bàn tay theo từng trục và xác nhận dấu (+/-) và
         * trục (roll/pitch) khớp đúng với chiều servo mong muốn ở B.2, trước
         * khi tin tưởng dùng cho Task_ControlLoop thật. Nếu sai, đổi công
         * thức bên dưới (đổi dấu hoặc hoán roll<->pitch) cho khớp lắp đặt
         * thật của IMU trên bàn, KHÔNG sửa lại phần cứng. */
        float roll_acc  = atan2f(ay, az) * (180.0f / (float)M_PI);
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);


        float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
        bool  trust_accel = (accel_norm > ACCEL_TRUST_MIN_G) && (accel_norm < ACCEL_TRUST_MAX_G);

        float vroll, vpitch;
        float roll, pitch;

        if (trust_accel) {
            /* Predict + Update đầy đủ - dùng accel để kéo góc về đúng, chống trôi */
            roll  = kalman1d_update(&kf_roll,  roll_acc,  gx, IMU_SAMPLE_DT_S, &vroll);
            pitch = kalman1d_update(&kf_pitch, pitch_acc, gy, IMU_SAMPLE_DT_S, &vpitch);
        } else {
            /* Bàn đang rung/dao động mạnh - CHỈ predict bằng gyro, bỏ qua accel
             * để không làm hỏng ước lượng góc bằng dữ liệu accel bị nhiễu động học.
             * Cách làm: gọi update với R_measure cực lớn tạm thời tương đương
             * "không update" - đơn giản hơn là viết thêm 1 hàm predict-only. */
            float saved_R = kf_roll.R_measure;
            kf_roll.R_measure  = 1.0e6f;
            kf_pitch.R_measure = 1.0e6f;
            roll  = kalman1d_update(&kf_roll,  roll_acc,  gx, IMU_SAMPLE_DT_S, &vroll);
            pitch = kalman1d_update(&kf_pitch, pitch_acc, gy, IMU_SAMPLE_DT_S, &vpitch);
            kf_roll.R_measure  = saved_R;
            kf_pitch.R_measure = saved_R;
        }

        //static uint32_t dbg_counter = 0;

        imu_state_write(system_state_get_imu_ptr(), roll, pitch, vroll, vpitch);
/*
        dbg_counter++;
        if (dbg_counter >= 100) {   // 500 chu kỳ x 1ms = 500ms, KHÔNG in mỗi 1kHz
            dbg_counter = 0;

            printf("%.2f,%.2f\r\n", roll, pitch);
            //printf("IMU_FUSION: roll=%.2f pitch=%.2f vroll=%.2f vpitch=%.2f\r\n", roll, pitch, vroll, vpitch);
        }
*/
        //printf("IMU_FUSION: roll=%.2f pitch=%.2f vroll=%.2f vpitch=%.2f\r\n", roll, pitch, vroll, vpitch);
        task_alive_mark(ALIVE_BIT_IMU_FUSION);
    }
}

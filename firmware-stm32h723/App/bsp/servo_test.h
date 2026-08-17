/**
 * @file    servo_test.h
 * @brief   Test mode doc lap cho B2 - xac nhan Tang 1/2/3 servo bang mat
 *          tren phan cung that, KHONG can IK (muc 5), KHONG can CAN/Jetson,
 *          KHONG can Task_Button_UI (chua co toi B.4).
 *
 * Code trong file nay la CODE TAM (giong pattern imu debug o B1) - se bi
 * xoa/thay the hoan toan khi ghep Task_ControlLoop that (B.6). Khi do chi
 * giu lai servo_actuator_step(dt) o cuoi chu ky, con lenh set_target/
 * apply_delta duoc thay bang output that cua IK/PID.
 *
 * Cach dung: goi servo_test_init() 1 lan luc khoi dong, roi goi
 * servo_test_step() moi 10ms (100Hz) trong vong lap StartControlLoop tam
 * (vd bang osDelay(10) hoac 1 TIM 100Hz rieng).
 *
 * Tham chieu: PingpongTable_ProfessionalDesign muc B.2.6, B.2.8.
 */
#ifndef SERVO_TEST_H
#define SERVO_TEST_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khoi tao actuator layer va trang thai test. Goi 1 lan luc dau
 *        StartControlLoop, TRUOC vong lap goi servo_test_step().
 */
void servo_test_init(void);

/**
 * @brief Chay 1 buoc test, PHAI goi dung nhip co dinh dt = 0.01f (100Hz).
 *        Tu dong chay tuan tu qua 4 giai doan (B.2.8):
 *          1. TEST_HOLD_NEUTRAL      - giu neutral 2s, xac nhan khong rung
 *          2. TEST_ABSOLUTE_SWEEP    - quet vi tri tuyet doi S1 (slew-rate)
 *          3. TEST_INCREMENTAL       - cong don incremental S2 (anti-windup)
 *          4. TEST_TRAJECTORY_HOME   - trajectory hinh thang S3 (ve neutral)
 *        roi lap lai tu dau. In log qua debug moi 0.5s.
 */
void servo_test_step(void);

/* ==========================================================================
 * PHẦN THÊM CHO GIAI ĐOẠN 3 (B6) — dùng bởi control_mode_manual.c
 * (App/control/), KHÔNG dùng bởi control_mode_calib.c (mode đó chỉ đọc kết
 * quả deadband đã đo sẵn qua control_mode_manual_get_deadband_result(),
 * xem B6_Control.md mục 0 + 5.3-5.4).
 * ========================================================================== */

typedef enum {
    SERVO_TEST_MODE_LEGACY_B2 = 0,     // servo_test_init()/servo_test_step() cũ ở trên
    SERVO_TEST_MODE_MANUAL_STEP,       // (1) chỉnh tay us từng servo, log roll/pitch
    SERVO_TEST_MODE_SWEEP_LOG,         // (2) quét toàn dải, log CSV
    /* SERVO_TEST_MODE_DEADBAND_SCAN đã BỎ - cảm biến nhiễu, đo không chính
     * xác, xem trao đổi Giai đoạn 4. Nếu sau này cần lại, làm phương pháp
     * khác (không dựa vào ngưỡng cứng so IMU). */
} servo_test_mode_t;

/**
 * @brief Bắt đầu 1 phiên test mới (Giai đoạn 3). Reset toàn bộ state nội bộ
 *        của phiên trước, gọi servo_test_log_csv_header() 1 lần.
 * @param mode      1 trong 3 mode mới ở trên (KHÔNG dùng LEGACY_B2 ở đây -
 *                  legacy dùng servo_test_init()/servo_test_step() riêng).
 * @param servo_ch  0 = quét/test tuần tự cả 3 servo (S1->S2->S3, dùng cho
 *                  DEADBAND_SCAN/SWEEP_LOG); 1/2/3 = chỉ 1 servo cụ thể
 *                  (dùng cho MANUAL_STEP, hoặc test riêng 1 trục).
 */
void servo_test_start(servo_test_mode_t mode, uint8_t servo_ch);

/** @brief Dừng phiên test hiện tại (nếu đang chạy), servo giữ nguyên vị trí. */
void servo_test_stop(void);

/** @brief true khi phiên DEADBAND_SCAN/SWEEP_LOG đã quét xong hết kênh yêu
 *         cầu. MANUAL_STEP không bao giờ tự "done" - luôn chờ lệnh dừng tay. */
bool servo_test_is_done(void);

/**
 * @brief Gọi mỗi chu kỳ Task_ControlLoop khi đang trong OPMODE_MANUAL,
 *        THAY cho servo_test_step() không tham số (bản legacy hard-code
 *        dt=0.01f nội bộ) - dt lấy từ chính Task_ControlLoop thật, không
 *        giả định tần số cố định.
 */
void servo_test_step_dt(float dt);

/**
 * @brief Chỉnh tay us cho kênh đang chọn (SERVO_TEST_MODE_MANUAL_STEP).
 *        Gọi từ task_button_ui.c (qua control_mode_manual_adjust()) khi
 *        người dùng bấm LEFT/RIGHT trên màn Manual. Tự log 1 dòng CSV mỗi
 *        lần gọi (mỗi lần bấm nút = 1 điểm dữ liệu roll/pitch tương ứng).
 */
void servo_test_manual_adjust(int16_t delta_us);

/** @brief Đổi kênh đang chỉnh tay (1/2/3) trong SERVO_TEST_MODE_MANUAL_STEP. */
void servo_test_manual_select_channel(uint8_t servo_ch);

/** @brief In dòng tiêu đề CSV 1 lần: "t_ms,S1_us,S2_us,S3_us,roll_deg,pitch_deg". */
void servo_test_log_csv_header(void);

/** @brief In 1 dòng CSV với giá trị S1/S2/S3/roll/pitch hiện tại. */
void servo_test_log_csv_row(uint32_t t_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_TEST_H */

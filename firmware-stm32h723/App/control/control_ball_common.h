#ifndef CONTROL_BALL_COMMON_H
#define CONTROL_BALL_COMMON_H
#include <stdbool.h>

/**
 * @file    control_ball_common.h
 * @brief   Inverse Kinematic (IK) + áp dụng vào servo - dùng chung cho
 *          Mode Balance (Mode 2) và Mode Position (Mode 3).
 *
 * SỬA (Giai đoạn 6 - khớp với control_mode_calib.c v5 + calibration_data v3):
 * mô hình Least Squares BẬC 1 (a,b,c / d,e,f tạm dùng field A1,B1,A2,B2,A3,B3)
 * đã bị thay bằng mô hình BẬC 2, vì thực nghiệm xác nhận pitch cong rõ theo
 * S1 - bậc 1 không đủ khớp trên toàn dải. Đồng thời dữ liệu quét giờ lấy từ
 * LƯỚI 2D (S1,S2) phủ đều mặt phẳng (roll,pitch) thay vì 3 đường quét
 * 1-trục, nên hệ số fit ra mới thật sự đáng tin trên toàn miền hoạt động
 * (không chỉ đúng dọc vài hướng như bản cũ).
 *
 * Công thức mới, cho mỗi servo i=0(S1),1(S2),2(S3):
 *
 *   S_i = ik_coef[i][0]*R + ik_coef[i][1]*P + ik_coef[i][2]*R^2
 *       + ik_coef[i][3]*P^2 + ik_coef[i][4]*R*P + ik_coef[i][5]
 *
 * R,P dùng ở đây LÀ roll_d,pitch_d truyền vào hàm - PHẢI là giá trị QUANH 0
 * (sai số/hiệu chỉnh PID, không phải góc thô tuyệt đối), đúng quy ước dữ
 * liệu calib đã trừ roll_offset/pitch_offset trước khi hồi quy (xem
 * control_mode_calib.c). KHÔNG được truyền góc thô vào hàm này.
 *
 * 18 hệ số (ik_coef[3][6]) được đo bằng control_mode_calib.c (lưới quét 2D
 * + Least Squares bậc 2 trên MCU) rồi lưu trực tiếp vào
 * calibration_data_t::ik_coef - KHÔNG còn dùng lại field A1,B1,A2,B2,A3,B3
 * cũ (calibration_data v3 đã đổi hẳn sang ik_coef[3][6], xem
 * calibration_data.h).
 *
 * S3 KHÔNG còn suy runtime bằng S3=-(S1+S2) nữa - hệ số ik_coef[2][*] đã
 * được TÍNH SẴN lúc calib sao cho luôn thỏa ràng buộc đó với MỌI (R,P)
 * (vì ràng buộc là tuyến tính trên hệ số, áp được ngay lúc fit - xem
 * solve_and_print_ik_coeffs() trong control_mode_calib.c). Việc này tránh
 * cộng dồn sai số làm tròn float giữa S1,S2,S3 mỗi tick control loop.
 *
 * Input:  roll_d, pitch_d (độ, quanh 0) - KHÔNG còn height_d/z ở TẦNG IK
 *         này (height được cộng RIÊNG, xem control_ball_apply_rph() bên
 *         dưới - Kế hoạch 2, không đụng vào ik_coef/control_ball_ik()).
 * Output: S1, S2, S3 - OFFSET so với neutral (KHÔNG phải xung PWM thật).
 *         Xung PWM thật = Sx_neutral + Sx (cộng ở control_ball_apply_rp(),
 *         KHÔNG cộng lại lần nữa ở nơi gọi).
 *
 * Hàm LUÔN có nghiệm hợp lệ (chỉ là đa thức bậc 2, không nghịch đảo ma
 * trận nào) - nếu ik_coef toàn 0 (chưa calib), kết quả luôn = 0 (giữ
 * neutral), an toàn giống hệt bản cũ.
 */

/**
 * @brief Tính IK từ (roll_d, pitch_d) ra offset S1/S2/S3 (chưa cộng
 *        neutral). Hàm THUẦN (không đọc/ghi trạng thái toàn cục nào khác
 *        ngoài calibration_data_get_ptr() để lấy ik_coef).
 */
void control_ball_ik(float roll_d, float pitch_d, float *s1, float *s2, float *s3);

/**
 * @brief Tính IK rồi áp thẳng vào servo qua servo_actuator_set_target()
 *        (offset + neutral đọc từ calibration_data). Dùng chung cho
 *        control_mode_balance.c và control_mode_position.c - 2 file đó tự
 *        chịu trách nhiệm tính ra roll_d/pitch_d rồi gọi hàm này, KHÔNG tự
 *        ý gọi servo_actuator_set_target() trực tiếp.
 *
 * THÊM (Kế hoạch 2): giờ là WRAPPER của control_ball_apply_rph(r, p, 0.0f)
 * - height_d=0 nghĩa là KHÔNG cộng thêm offset chiều cao nào, hành vi
 * giống hệt bản cũ 100%. Mọi nơi đang gọi hàm này (vd control_mode_position.c)
 * KHÔNG cần sửa gì cả.
 */
void control_ball_apply_rp(float roll_d, float pitch_d);

/* ===========================================================================
 * Kế hoạch 2 - Height Control (tâng bóng), THÊM MỚI.
 *
 * Cơ chế: offset (us) cộng ĐỀU vào cả 3 servo, ĐỘC LẬP hoàn toàn với IK
 * roll/pitch (không đụng ik_coef/control_ball_ik() ở trên). Dấu ÂM trong
 * công thức = giảm us = NÂNG bàn (quy ước: height_d dương = nâng bàn,
 * height_d âm = hạ bàn - khớp với comment Height_d trong CAN 0x204).
 *
 * HỆ SỐ (đã đo thực tế, bản ĐỐI XỨNG mới nhất - THAY bản lệch nâng/hạ cũ
 * 18.125/17.935 us/mm trong bản nháp plan ban đầu):
 *   S1/S2/S3 giảm 290us <-> bàn cao thêm 13.0mm
 *   S1/S2/S3 tăng 290us <-> bàn hạ xuống 13.0mm
 *   => CHỈ 1 hệ số duy nhất: K = 290/13.0 ~= 22.3077 us/mm, áp dụng NHƯ
 *      NHAU cho cả 2 chiều (khác bản cũ có 2 hệ số lệch nhau).
 * =========================================================================== */

/* Giới hạn an toàn height_d (mm) - PHẢI clamp height_d vào khoảng này
 * TRƯỚC khi tính offset, vì ngoài khoảng này CHƯA có dữ liệu xác nhận
 * tuyến tính thực tế (an toàn, tránh ngoại suy sai). Đối xứng theo hệ số
 * đối xứng đã đo ở trên.
 * TODO: nếu sau này đo thêm xác nhận tuyến tính xa hơn 13mm (1 chiều hoặc
 * cả 2), sửa lại đúng 2 hằng số này - KHÔNG sửa ở đâu khác. */
#define HEIGHT_D_MIN_MM (-13.0f)
#define HEIGHT_D_MAX_MM (13.0f)

/* Giới hạn TỔNG offset mỗi trục (us) - phần cứng đã xác nhận an toàn tối
 * đa ±360us/trục ở mode Calib. Áp dụng cho TỔNG (đóng góp roll/pitch từ
 * IK + đóng góp height), KHÔNG clamp riêng từng thành phần trước khi
 * cộng - clamp riêng rồi cộng có thể vẫn vượt 360us sau khi cộng, sai
 * đúng yêu cầu an toàn đã chốt trong plan (mục 2.2). */
#define AXIS_OFFSET_MAX_US (360.0f)

/**
 * @brief Chuyển height_d (mm, đã clamp vào [HEIGHT_D_MIN_MM,
 *        HEIGHT_D_MAX_MM] BÊN TRONG hàm) thành offset us cộng đều vào cả
 *        3 servo. Hàm THUẦN, không đọc/ghi trạng thái toàn cục.
 *        Public chủ yếu để unit-test/debug in ra giá trị - luồng bình
 *        thường không cần gọi trực tiếp, dùng control_ball_apply_rph().
 */
float control_ball_height_offset_us(float height_d_mm);

/**
 * @brief Như control_ball_apply_rp() nhưng CỘNG THÊM offset chiều cao
 *        (control_ball_height_offset_us(height_d)) vào TỔNG offset mỗi
 *        trục, rồi clamp TỔNG đó vào ±AXIS_OFFSET_MAX_US trước khi áp
 *        vào servo. Dùng cho Mode Balance khi có Height_d hợp lệ từ
 *        Jetson (0x204, setpoint_t.Height_d).
 */
void control_ball_apply_rph(float roll_d, float pitch_d, float height_d);

#endif /* CONTROL_BALL_COMMON_H */

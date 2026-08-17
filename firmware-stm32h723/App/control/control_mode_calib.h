#ifndef CONTROL_MODE_CALIB_H
#define CONTROL_MODE_CALIB_H
#include <stdbool.h>
#include <stdint.h>

/**
 * @file    control_mode_calib.h
 * @brief   Mode 1 - Calib PHIÊN BẢN 5: lưới quét 2D + Least Squares BẬC 2.
 *
 * SỬA (Giai đoạn 6 - khắc phục 2 lỗi thiết kế của bản v4):
 *
 * (1) THIẾT KẾ QUÉT SAI: v4 quét 3 vòng, mỗi vòng chỉ đổi 1 servo "chính"
 *     (servo kia bù cố định -t/2) - dữ liệu (roll,pitch) thu được CHỈ nằm
 *     dọc 3 ĐƯỜNG THẲNG qua gốc (ứng với hướng nghiêng riêng của từng
 *     servo), KHÔNG phải lưới phủ đều mặt phẳng (roll,pitch). Hồi quy fit
 *     trên 3 đường thẳng có thể cho R² cao (vì đúng trên chính 3 đường đó)
 *     nhưng SAI nặng ở các điểm ngoài 3 đường - tức hầu hết trường hợp thực
 *     tế khi PID Balance đang chỉnh roll/pitch lệch đồng thời theo hướng
 *     bất kỳ. SỬA: quét LƯỚI 2D độc lập (S1,S2), CALIB_GRID_POINTS x
 *     CALIB_GRID_POINTS điểm, S3=-(S1+S2) (giữ đúng ràng buộc cơ khí), phủ
 *     đều toàn miền hoạt động thực tế thay vì 3 đường.
 *
 * (2) BẬC MÔ HÌNH SAI: v4 dùng S_i = a*R + b*P + c (bậc 1, phẳng). Dữ liệu
 *     thực nghiệm đã cho thấy pitch cong rõ theo S1 - mô hình bậc 1 KHÔNG
 *     đủ khớp. SỬA: nâng lên BẬC 2 (thêm R², P², R*P):
 *         S_i = c0*R + c1*P + c2*R^2 + c3*P^2 + c4*R*P + c5
 *     Giải bằng Least Squares tổng quát (ma trận chuẩn tắc 6x6, khử Gauss
 *     có chọn trục xoay - pivoting), KHÔNG dùng Cramer viết tay như v4
 *     (Cramer 6x6 quá cồng kềnh, dễ sai khi mở rộng tay).
 *
 * (3) MỚI - GATE TỰ ĐỘNG DỰA TRÊN CHẤT LƯỢNG FIT: v4 đã TÍNH RMS/max_err/R²
 *     nhưng CHƯA DÙNG để quyết định có lưu Flash hay không (s_ls_ok chỉ
 *     phụ thuộc "giải được hệ toán học" chứ không phụ thuộc "fit có tốt
 *     không"). SỬA: s_ls_ok giờ = (giải được) AND (rms <= ngưỡng) AND
 *     (max_err <= ngưỡng) cho CẢ 2 phương trình S1,S2 - nếu fit tệ (dữ
 *     liệu nhiễu, servo kẹt, IMU lỗi...) sẽ TỪ CHỐI ghi Flash, giữ nguyên
 *     hệ số cũ, thay vì âm thầm lưu 1 mô hình không đáng tin.
 *
 * Quy trình:
 *   1. OFFSET      : về neutral -> settle 2s -> trung bình 2s (200 mẫu) ->
 *                    roll_offset, pitch_offset
 *   2. SWEEP_GRID  : quét lưới (S1,S2) CALIB_GRID_POINTS x CALIB_GRID_POINTS
 *                    điểm quanh neutral (S3 = -(S1+S2)), mỗi điểm: di
 *                    chuyển -> settle -> trung bình CALIB_SAMPLE_COUNT mẫu
 *                    -> IN 1 dòng CSV "S1 S2 S3 roll pitch" (không về
 *                    neutral giữa các điểm - quét liên tục cho nhanh), sau
 *                    điểm cuối cùng: giải Least Squares bậc 2 NGAY TRÊN MCU.
 *   3. SAVE        : lưu roll_offset/pitch_offset VÀ 18 hệ số bậc 2
 *                    (ik_coef[3][6]) vào Flash - CHỈ ghi nếu Least Squares
 *                    thành công VÀ đạt ngưỡng sai số cho phép, nếu không
 *                    giữ nguyên hệ số cũ (an toàn hơn ghi giá trị tệ).
 *
 * Dữ liệu (R,P) dùng để giải Least Squares là roll/pitch ĐÃ TRỪ
 * roll_offset/pitch_offset (delta so với mặt phẳng cân bằng, quanh 0) -
 * khớp đúng quy ước control_mode_balance.c đưa vào IK giá trị PID output
 * (sai số quanh 0), không phải góc thô tuyệt đối. Lý do giữ nguyên như v4,
 * xem lại lịch sử sửa lỗi "servo nhảy ngay khi vào Balance" nếu cần.
 *
 * KHÔNG bao giờ tự gọi system_state_publish() - chỉ gửi EVT_MODE_CALIB_DONE/
 * EVT_MODE_CALIB_FAILED qua StateRequestQueueHandle.
 */

typedef enum {
    CALIB_SUB_OFFSET = 0,
    CALIB_SUB_SWEEP_GRID,   /* SỬA - thay thế 3 state SWEEP_S1/S2/S3 của v4 */
    CALIB_SUB_SAVE,
    CALIB_SUB_DONE,
    CALIB_SUB_ERROR
} calib_sub_state_t;

void control_mode_calib_enter(void);
void control_mode_calib_step(float dt);
calib_sub_state_t control_mode_calib_get_sub_state(void);

/* MỚI - cho phép caller (UI/menu chỉnh neutral) ĐẶT LẠI S1/S2/S3_neutral
 * TRƯỚC khi chạy lại Mode Calib, thay vì luôn giữ neutral cũ đọc từ Flash.
 *
 * PHẢI gọi hàm này SAU control_mode_calib_enter() (enter() sẽ xoá override
 * cũ nếu có, coi như "mặc định không đổi neutral" mỗi lần vào Mode Calib
 * mới) và TRƯỚC lần control_mode_calib_step() đầu tiên - vì neutral mới sẽ
 * được dùng làm TÂM của cả bước OFFSET lẫn SWEEP_GRID (không chỉ lúc ghi
 * Flash ở SAVE), để dữ liệu quét/offset đo đúng quanh vị trí neutral mới,
 * không lệch so với neutral cũ.
 *
 * Sau khi Mode Calib chạy xong (SAVE thành công hay thất bại), override bị
 * xoá tự động - lần chạy Mode Calib kế tiếp mặc định dùng lại neutral hiện
 * có trong Flash trừ khi gọi lại hàm này. */
void control_mode_calib_set_neutral(int16_t s1_neutral, int16_t s2_neutral, int16_t s3_neutral);

#endif /* CONTROL_MODE_CALIB_H */

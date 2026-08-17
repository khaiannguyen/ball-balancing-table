#ifndef CALIBRATION_DATA_H
#define CALIBRATION_DATA_H
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * calibration_data.h — PHIÊN BẢN 3 (Giai đoạn 6 - IK bậc 2 + lưới quét 2D)
 *
 * SỬA so với v2:
 *  - v2 dùng 6 hệ số A1,B1,A2,B2,A3,B3 cho mô hình BẬC 1 (S_i = A_i*R + B_i*P
 *    + const, giải bằng Cramer 3x3). Dữ liệu quét v2 lấy từ 3 sweep 1-trục
 *    (mỗi lần chỉ đổi 1 servo, 2 servo còn lại bù cố định -t/2) - phần dữ
 *    liệu này CHỈ phủ 3 đường thẳng qua gốc trong mặt phẳng (roll,pitch),
 *    không đủ đa dạng để tin cậy ngoài 3 đường đó. Thực nghiệm đã xác nhận
 *    pitch cong rõ theo S1 -> mô hình bậc 1 KHÔNG đủ khớp trên toàn dải.
 *  - v3: đổi sang LƯỚI QUÉT 2D (S1,S2) độc lập (S3=-(S1+S2)), phủ đều toàn
 *    miền hoạt động thực tế, và MÔ HÌNH BẬC 2:
 *        S_i = c0*R + c1*P + c2*R^2 + c3*P^2 + c4*R*P + c5
 *    (6 hệ số/servo, 18 hệ số tổng - thay thế hoàn toàn A1,B1,A2,B2,A3,B3).
 *    Lưu CẢ 3 servo (không suy S3 = -(S1+S2) lúc chạy control loop nữa) để
 *    tránh cộng dồn sai số làm tròn float mỗi tick - S3 vẫn được fit bằng
 *    ràng buộc S3=-(S1+S2) áp lên hệ số ngay lúc giải Least Squares (xem
 *    control_mode_calib.c), chỉ khác là kết quả được LƯU tường minh.
 *
 * Đổi CALIB_MAGIC (v2->v3) để KHÔNG BAO GIỜ nạp nhầm struct cũ (field
 * A1,B1... nghĩa hoàn toàn khác, layout khác) - buộc chạy lại Mode Calib.
 * ========================================================================== */

#define CALIB_MAGIC     0xC0FFEE03u   /* v2 (0xC0FFEE02) -> v3: struct đổi hoàn toàn */
#define CALIB_VERSION   3u

/* Biên độ nghiêng tối đa dùng cho min/max servo (giữ nguyên như v2). Đây là
 * giới hạn AN TOÀN CƠ KHÍ tuyệt đối, KHÔNG phải biên độ quét calib (biên độ
 * quét calib giờ dùng CALIB_GRID_HALF_US riêng trong control_mode_calib.c,
 * nhỏ hơn số này để luôn có margin an toàn khi S1,S2 cộng dồn). */
#define CALIB_TILT_MAX_US   360

/* Số hệ số bậc 2 cho mỗi servo: [R, P, R^2, P^2, R*P, hằng số]. */
#define CALIB_IK_NUM_COEF   6

typedef struct {
    uint32_t magic;               // CALIB_MAGIC - nhận diện dữ liệu hợp lệ
    uint16_t version;             // CALIB_VERSION
    uint16_t _reserved0;          // padding tường minh

    int16_t  S1_neutral, S2_neutral, S3_neutral;   // đo thực nghiệm (mặc định 1576/1528/1536)
    int16_t  S1_max, S2_max, S3_max;                 // = neutral + CALIB_TILT_MAX_US (tự tính khi save)
    int16_t  S1_min, S2_min, S3_min;                 // = neutral - CALIB_TILT_MAX_US (tự tính khi save)

    float    roll_offset, pitch_offset;   // bù sai số lắp IMU, đo tại neutral

    /* MỚI (v3) - hệ số IK BẬC 2 cho từng servo i=0(S1),1(S2),2(S3):
     *   S_i = ik_coef[i][0]*R + ik_coef[i][1]*P + ik_coef[i][2]*R^2
     *       + ik_coef[i][3]*P^2 + ik_coef[i][4]*R*P + ik_coef[i][5]
     * R,P LÀ roll/pitch ĐÃ TRỪ roll_offset/pitch_offset (quanh 0, đúng quy
     * ước PID output mà control_mode_balance.c đưa vào IK - xem chú thích
     * chi tiết trong control_mode_calib.c, không lặp lại ở đây).
     * ik_coef = tất cả 0 -> IK luôn ra S_i=0 (giữ neutral) nếu chưa calib -
     * AN TOÀN, giống hệt nguyên tắc A_i/B_i=0 của v2. */
    float    ik_coef[3][CALIB_IK_NUM_COEF];

    int16_t  deadband_S1, deadband_S2, deadband_S3;   // đo bằng Mode Manual, giữ nguyên như v2
    int16_t  _reserved1;           // padding tường minh, giữ struct chẵn 4-byte trước crc32

    uint32_t crc32;                // CRC32 phần cứng, tính trên toàn bộ struct TRỪ field crc32 này
} calibration_data_t;

/* Gọi 1 lần lúc INIT (trước khi cho hệ vào STATE_READY). Trả false nếu
 * magic/version/crc sai -> gọi calibration_data_is_valid() để quyết định
 * có bắt buộc STATE_CALIBRATION hay không. */
bool calibration_data_load(void);

/* CHỈ gọi từ control_mode_calib.c :: CALIB_SUB_SAVE. Tự set magic/version,
 * tự tính lại S1/2/3_min/max = neutral ± CALIB_TILT_MAX_US, tự tính crc32
 * trước khi ghi Flash. Trả false nếu ghi Flash thất bại. */
bool calibration_data_save(const calibration_data_t *in);

/* Con trỏ read-only tới bản RAM hiện tại. KHÔNG cần seqlock: ghi chỉ xảy ra
 * trong STATE_CALIBRATION (loại trừ với mọi state khác đang đọc). */
const calibration_data_t *calibration_data_get_ptr(void);

/* true nếu bản RAM hiện tại có magic+version+crc hợp lệ. */
bool calibration_data_is_valid(void);

/* Giá trị mặc định dùng khi Flash chưa có / dữ liệu hỏng - AN TOÀN: mọi
 * ik_coef[i][*] = 0 nghĩa là IK sẽ luôn ra S1=S2=S3=0 (giữ neutral) nếu vô
 * tình chạy Balance/Position trước khi calib xong, KHÔNG di chuyển sai. */
void calibration_data_set_defaults(calibration_data_t *out);

void calibration_data_apply_to_actuator(void);

/* Giữ nguyên như v2 - xem giải thích trong calibration_data.c. */
void calibration_data_iwdg_widen_for_boot(void);
void calibration_data_iwdg_restore_orig(void);

#endif /* CALIBRATION_DATA_H */

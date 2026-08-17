#include "control_mode_home.h"
#include "servo_actuator.h"
#include "calibration_data.h"
#include "system_state.h"
#include "task_state_machine.h"   // state_event_t, EVT_HOME_DONE
#include "trajectory.h"           // MỚI - trajectory_start_synced3/update/is_done
#include "cmsis_os2.h"
#include <stdlib.h>               // abs()
#include <math.h>                 // lroundf()

extern osMessageQueueId_t StateRequestQueueHandle;   // khai báo trong main.c (mục 3.6 B5)

/* Dung sai + debounce - giá trị khởi điểm theo B6_Control.md mục 4, PHẢI
 * tinh chỉnh lại bằng thực nghiệm khi có bàn thật (bù rung nhiễu đọc PWM
 * và độ chính xác cơ khí thật). */
#define HOME_TOLERANCE_US    5
#define HOME_SETTLE_CYCLES   20   // ở CONTROL_LOOP_DT_S=0.01f -> ~200ms ổn định

/* SỬA - tính theo datasheet HPS-2018 (servo_actuator.c: SERVO_SLEW_MAX_US_PER_S
 * = 3200.0f, ~67% tốc độ lý thuyết 4762us/s = (60/0.14deg/s)*(2000us/180deg),
 * margin an toàn tránh dòng sát stall current 1.4-2.3A theo datasheet).
 * V_MAX đặt THẤP HƠN MỘT CHÚT so với 3200 (không đặt bằng hoặc cao hơn) để
 * trajectory luôn là ràng buộc chính, slew limiter chỉ còn là lưới an toàn
 * dự phòng KHÔNG BAO GIỜ thực sự cắt bớt (nếu đặt bằng/cao hơn, sai số làm
 * tròn dt hoặc chu kỳ dao động có thể khiến slew limiter thỉnh thoảng vẫn
 * cắt, phá đồng bộ 3 trục). A_MAX không có trong datasheet (servo hobby
 * không công bố gia tốc/jerk) - chọn để pha tăng tốc ngắn (~0.375s) nhưng
 * không giật cứng ngay từ đầu; CẦN xác nhận lại bằng thực nghiệm nếu thấy
 * cơ khí rung/giật lúc bắt đầu di chuyển. */
#define HOME_TRAJ_V_MAX_US_S    3000.0f
#define HOME_TRAJ_A_MAX_US_S2   8000.0f

static bool     s_entered   = false;   // đã khởi tạo trajectory cho lần vào Home hiện tại chưa
static bool     s_evt_sent  = false;   // đã gửi EVT_HOME_DONE thành công chưa (tránh gửi lặp)
static uint16_t s_settle_ct = 0;       // đếm số chu kỳ liên tiếp nằm trong dung sai

static trajectory_t s_traj[3];         // MỚI - 1 trajectory riêng cho mỗi servo S1/S2/S3
static bool          s_traj_done = false;   // MỚI - cả 3 trajectory đã tới đích setpoint chưa

void control_mode_home_enter(void)
{
    const calibration_data_t *c = calibration_data_get_ptr();

    /* Đọc vị trí THẬT hiện tại làm điểm xuất phát - BẮT BUỘC, nếu dùng lại
     * setpoint cũ (không phải vị trí thật) trajectory sẽ tính sai quãng
     * đường/thời gian (bài học B2 mục 6, nhắc lại trong trajectory.h). */
    int32_t s1, s2, s3;
    servo_actuator_get_local(&s1, &s2, &s3);

    float from[3] = { (float)s1, (float)s2, (float)s3 };
    float to[3]   = { (float)c->S1_neutral, (float)c->S2_neutral, (float)c->S3_neutral };

    /* SỬA (Giai đoạn 5): thay vì set_target() 1 lần rồi phó mặc hoàn toàn
     * cho slew-rate limiter, giờ chủ động sinh profile hình thang ĐỒNG BỘ
     * cho cả 3 servo - trục nào quãng đường ngắn hơn sẽ tự động đi CHẬM
     * hơn (v_max/a_max bị scale xuống) để cùng tới đích lúc
     * T = max(T0_1, T0_2, T0_3), thay vì trục ngắn xong trước, trục dài
     * xong sau (kiểu cũ chạy độc lập slew-rate limiter không đồng bộ). */
    trajectory_start_synced3(s_traj, from, to, HOME_TRAJ_V_MAX_US_S, HOME_TRAJ_A_MAX_US_S2);

    s_entered   = true;
    s_evt_sent  = false;
    s_settle_ct = 0;
    s_traj_done = false;
}

void control_mode_home_step(float dt)
{
    if (!s_entered) {
        /* An toàn: nếu Task_ControlLoop lỡ không gọi enter() riêng (vd quên
         * check mode_changed), tự vào đây 1 lần thay vì đứng yên vô thời hạn. */
        control_mode_home_enter();
    }

    if (s_evt_sent) {
        /* Đã báo xong, chờ Task_StateMachine chuyển state (RUN->READY) -
         * Task_ControlLoop sẽ tự ngừng gọi hàm này ở chu kỳ kế tiếp vì
         * system_state_get() != STATE_RUN nữa. Không làm gì thêm ở đây. */
        return;
    }

    /* MỚI (Giai đoạn 5): chạy trajectory mỗi chu kỳ cho tới khi cả 3 trục
     * báo xong, thay vì chỉ set_target 1 lần lúc enter(). Sau khi
     * s_traj_done == true, KHÔNG gọi set_target() nữa - giữ nguyên target
     * cuối cùng (đã là neutral), tránh ghi đè không cần thiết. */
    if (!s_traj_done) {
        /* SUA: trajectory_update() doi chu ky (nhan them dt + tra ve qua
         * con tro x_ref/v_ref/a_ref, thay vi return float truc tiep) - xem
         * trajectory.h ban moi. Home khong can v_ref/a_ref nen truyen NULL. */
        float p1, p2, p3;
        trajectory_update(&s_traj[0], dt, &p1, NULL, NULL);
        trajectory_update(&s_traj[1], dt, &p2, NULL, NULL);
        trajectory_update(&s_traj[2], dt, &p3, NULL, NULL);

        servo_actuator_set_target(SERVO_CH_S1, (int32_t)lroundf(p1));
        servo_actuator_set_target(SERVO_CH_S2, (int32_t)lroundf(p2));
        servo_actuator_set_target(SERVO_CH_S3, (int32_t)lroundf(p3));

        if (trajectory_is_done(&s_traj[0]) &&
            trajectory_is_done(&s_traj[1]) &&
            trajectory_is_done(&s_traj[2])) {
            s_traj_done = true;
        }
    }

    /* Debounce/settle GIỮ NGUYÊN như bản cũ - vẫn cần thiết như 1 lớp xác
     * nhận ĐỘC LẬP với trajectory: "trajectory báo done" chỉ có nghĩa là
     * SETPOINT đã tới đích, không chắc chắn vị trí THẬT của servo (đọc qua
     * servo_actuator_get_local()) đã hết dao động dư - đặc biệt nếu slew
     * limiter còn nắn lại tín hiệu phía sau trajectory. */
    const calibration_data_t *c = calibration_data_get_ptr();
    int32_t s1, s2, s3;
    servo_actuator_get_local(&s1, &s2, &s3);

    bool at_home =
        (abs((int)(s1 - c->S1_neutral)) < HOME_TOLERANCE_US) &&
        (abs((int)(s2 - c->S2_neutral)) < HOME_TOLERANCE_US) &&
        (abs((int)(s3 - c->S3_neutral)) < HOME_TOLERANCE_US);

    s_settle_ct = at_home ? (uint16_t)(s_settle_ct + 1) : 0;

    if (s_settle_ct >= HOME_SETTLE_CYCLES) {
        state_event_t evt = EVT_HOME_DONE;
        /* timeout=0: KHÔNG được block control loop chờ queue trống.
         * StateRequestQueueHandle size 16 (main.c) nên gần như luôn thành
         * công; nếu vì lý do gì đó đầy, đơn giản thử lại chu kỳ sau (giữ
         * s_settle_ct nguyên, không reset) thay vì mất event vĩnh viễn. */
        osStatus_t st = osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
        if (st == osOK) {
            s_evt_sent = true;
        }
        /* st != osOK: không làm gì, control_mode_home_step() sẽ thử gửi lại
         * ở chu kỳ kế tiếp vì s_evt_sent vẫn false và s_settle_ct vẫn >= HOME_SETTLE_CYCLES. */
    }
}

bool control_mode_home_is_done(void)
{
    return s_evt_sent;
}

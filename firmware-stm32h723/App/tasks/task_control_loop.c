#include "task_control_loop.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* ---- Init/driver cũ, giữ nguyên như code đang chạy trong main.c ---- */
#include "imu_mpu6500.h"
#include "buttons.h"

/* ---- Servo layer (B2, đã có sẵn) ---- */
#include "servo_actuator.h"

/* ---- State/setpoint/calib (B5 + Giai đoạn 1 B6) ---- */
#include "system_state.h"
#include "calibration_data.h"

/* ---- Control mode (Giai đoạn 2-3 B6) ---- */
#include "control_mode_home.h"
#include "control_mode_manual.h"
#include "control_mode_calib.h"
#include "control_mode_balance.h"
#include "control_mode_position.h"

#include "screen_boot.h"   /* THÊM (B7): ScreenBoot_AddLog() - log từng bước init lên màn Boot */

/* TODO Giai đoạn 5: #include "control_mode_balance.h", "control_mode_position.h" */

extern SPI_HandleTypeDef hspi2;   // dùng cho imu_mpu6500_init(), như code cũ trong main.c
extern osEventFlagsId_t   SystemEventGroupHandle;   /* THÊM (B7): đã tồn tại sẵn, dùng chung
                                                        cho EVT_BIT_FAULT - giờ thêm EVT_BIT_BOOT_DONE */
extern osMessageQueueId_t StateRequestQueueHandle;  /* THÊM (B7): gửi EVT_SELFTEST_OK/FAIL,
                                                        EVT_CALIB_DONE/FAIL thay TEMP-1 trong
                                                        task_state_machine.c */

#define CONTROL_LOOP_DT_S       0.01f   // 100Hz - PHẢI khớp osDelay bên dưới
#define CONTROL_LOOP_PERIOD_MS  10

void StartTaskControlLoop(void *argument)
{
    (void)argument;

    /* THÊM (quan trọng - fix MCU reset liên tục lúc boot, cách chắc chắn
     * nhất): nới IWDG lên ~32s NGAY DÒNG ĐẦU TIÊN của task, TRƯỚC cả đoạn
     * chờ EVT_BIT_TFT_READY - dùng đúng kỹ thuật ghi thanh ghi IWDG trực
     * tiếp đã áp dụng thành công cho calibration_data_save() lúc ghi Flash.
     * Lý do chọn cách này thay vì chỉ dựa vào task_alive_mark() rải rác:
     * chưa rõ Task_Watchdog có yêu cầu cả 3 bit (CONTROL_LOOP|IMU_FUSION|
     * CAN_RX) phải đến trong CÙNG 1 cửa sổ ngắn hay không - nếu đúng vậy,
     * dù Task_ControlLoop tự mark liên tục, Task_IMU_Fusion/Task_CAN_RX có
     * thể chưa kịp chạy đủ trong lúc Task_ControlLoop bận init/chờ TFT/ghi
     * log Boot -> vẫn reset. Nới trực tiếp thanh ghi IWDG loại bỏ hoàn
     * toàn khả năng này, không phụ thuộc logic Task_Watchdog là gì. PHẢI
     * gọi calibration_data_iwdg_restore_orig() NGAY TRƯỚC khi vào vòng lặp
     * chính for(;;) bên dưới - không để watchdog bị nới lỏng vĩnh viễn. */
    calibration_data_iwdg_widen_for_boot();

    /* THÊM (B7, fix deadlock màn Boot): chờ Task_Display báo TFT đã init +
     * khung Boot đã vẽ xong (EVT_BIT_TFT_READY) TRƯỚC khi gọi bất kỳ
     * ScreenBoot_AddLog() nào bên dưới - xem giải thích đầy đủ trong
     * system_state.h. Timeout 2000ms (không osWaitForever): nếu vì lý do
     * gì đó Task_Display không bao giờ set bit này (vd lỗi khác trong
     * TFT_Init), Task_ControlLoop vẫn phải tiếp tục init IMU/servo - đây
     * là task quan trọng nhất của hệ thống, không được phép treo vô hạn
     * chỉ vì màn hình. osEventFlagsWait timeout trả osFlagsError, bỏ qua
     * lỗi đó và chạy tiếp bình thường (ScreenBoot_AddLog gọi sau vẫn có
     * thể lỗi/không hiện gì, nhưng không làm treo toàn hệ thống). */
    /* SỬA (quan trọng - fix MCU tự reset liên tục lúc boot): KHÔNG được
     * chờ EVT_BIT_TFT_READY 1 lần liên tục (dù chỉ 2000ms) - task_alive_mark
     * (ALIVE_BIT_CONTROL_LOOP) chỉ được gọi BÊN TRONG vòng lặp for(;;) phía
     * dưới, nên suốt thời gian chờ này Task_Watchdog không thấy CONTROL_LOOP
     * "còn sống" -> IWDG (mặc định ~0.5s) bắn giữa chừng trước khi vào được
     * vòng lặp chính -> MCU reset liên tục ngay sau màn Boot, không bao giờ
     * tới Home (đúng triệu chứng đã gặp). Sửa: chia nhỏ thời gian chờ thành
     * từng đoạn NGẮN (200ms), gọi task_alive_mark() SAU MỖI đoạn - vẫn giữ
     * tổng thời gian chờ tối đa ~2000ms, nhưng không còn khoảng "im lặng"
     * nào dài hơn watchdog timeout. */
    #define TFT_READY_POLL_MS       200u
    #define TFT_READY_TOTAL_MS      2000u
    {
        uint32_t poll_ticks = (TFT_READY_POLL_MS * osKernelGetTickFreq()) / 1000u;
        uint32_t waited_ms  = 0u;
        uint32_t flags;
        do {
            flags = osEventFlagsWait(SystemEventGroupHandle, EVT_BIT_TFT_READY,
                                      osFlagsWaitAny, poll_ticks);
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);   /* báo sống mỗi 200ms trong lúc chờ */
            if ((flags & osFlagsError) == 0) break;    /* đã nhận được bit -> thoát ngay */
            waited_ms += TFT_READY_POLL_MS;
        } while (waited_ms < TFT_READY_TOTAL_MS);
    }

    /* ---- Phần init giữ NGUYÊN như code cũ đang chạy trong main.c ---- */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    bool imu_ok = imu_mpu6500_init(&hspi2);
    printf("MPU6500 init: %s\r\n", imu_ok ? "OK" : "FAIL");
    ScreenBoot_AddLog(imu_ok ? "IMU MPU6500: OK" : "IMU MPU6500: FAIL");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);   /* THÊM: phòng ScreenBoot_AddLog chờ mutex lâu */

    buttons_init();
    ScreenBoot_AddLog("Buttons: OK");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /* Giai đoạn 1: nạp calib từ Flash lúc INIT. Nếu Flash chưa có/hỏng,
     * calibration_data_load() tự nạp giá trị default an toàn (neutral=1576
     * theo thực nghiệm) vào RAM và trả false - đủ để Mode Home chạy tạm
     * trước khi calib thật. calibration_data_is_valid() sẽ được nối vào
     * guard STATE_READY ở Giai đoạn 4 (chưa làm ở đây). */
    bool calib_load_ok = calibration_data_load();
    ScreenBoot_AddLog(calib_load_ok ? "Calib data: LOADED" : "Calib data: DEFAULT (chua calib)");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /* SỬA (B7): XOÁ khối servo_actuator_set_calib() thủ công từng gọi ở đây
     * trước calibration_data_apply_to_actuator() - 2 đoạn code làm cùng 1
     * việc 2 lần, nhưng calibration_data_apply_to_actuator() từng dùng
     * hằng số deadband hard-code khác với field s_calib.deadband_S1/2/3
     * (đã sửa trong calibration_data.c cùng đợt này). Giờ CHỈ còn 1 nguồn
     * áp calib xuống servo_actuator, tránh lệch nhau nếu sau này 1 trong 2
     * chỗ bị sửa mà quên chỗ kia.
     *
     * BẮT BUỘC gọi TRƯỚC servo_actuator_init() để giá trị pos_us/target_us
     * khởi tạo ban đầu (bên trong init()) lấy đúng neutral thật
     * (1576/1528/1536) thay vì mặc định cũ B2 (1500) - nếu gọi sau,
     * init() đã lỡ chốt pos_us=1500 rồi mới đổi bảng calib. */
    calibration_data_apply_to_actuator();
    ScreenBoot_AddLog("Calib -> Servo: applied");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /* THAY servo_test_init() cũ bằng servo_actuator_init() trực tiếp -
     * servo_test_init() bên trong cũng chỉ gọi servo_actuator_init(), nhưng
     * Task_ControlLoop không còn tự chạy servo_test mặc định nữa. Từ Giai
     * đoạn 3, servo_test (API mới servo_test_start/step_dt) chỉ được gọi
     * gián tiếp qua control_mode_manual.c khi setpoint.mode == OPMODE_MANUAL. */
    servo_actuator_init();
    ScreenBoot_AddLog("Servo actuator: OK");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /* THÊM (B7): self-test thật thay TEMP-1 trong task_state_machine.c.
     * imu_ok: kết quả init IMU ở trên. calib_load_ok/calibration_data_is_valid():
     * true nếu Flash có calib hợp lệ (magic+version+crc đúng) - KHÔNG cần
     * calib lại mỗi lần boot, chỉ cần calib đã lưu từ lần chạy Mode Calib
     * trước đó còn tốt. Nếu IMU fail -> EVT_SELFTEST_FAIL -> STATE_ERROR
     * ngay, không cho chạy tiếp với IMU chết. Nếu IMU OK nhưng calib chưa
     * từng chạy/hỏng -> EVT_CALIB_FAIL -> STATE_ERROR, buộc người dùng
     * chạy Mode Calib trước khi dùng Balance/Position (IK an toàn trả về
     * neutral nếu A_i/B_i=0, nhưng vẫn nên chặn ở state machine cho rõ ràng). */
    {
        state_event_t evt;
        if (imu_ok) {
            evt = EVT_SELFTEST_OK;
            osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
            ScreenBoot_AddLog("Self-test: OK");
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

            bool calib_valid = calibration_data_is_valid();
            evt = calib_valid ? EVT_CALIB_DONE : EVT_CALIB_FAIL;
            osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
            ScreenBoot_AddLog(calib_valid ? "Calib: VALID" : "Calib: INVALID - can calib lai!");
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
        } else {
            evt = EVT_SELFTEST_FAIL;
            osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
            ScreenBoot_AddLog("Self-test: FAIL (IMU)");
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
        }
        (void)calib_load_ok;  /* dùng calibration_data_is_valid() làm nguồn sự thật, giữ biến này chỉ để log nếu cần */
    }

    ScreenBoot_AddLog("READY");

    /* THÊM (B7): báo Task_Display init xong (IMU + calib + servo + self-test),
     * thay osDelay(2000) cứng trong task_display.c. Đặt SAU MỌI log/self-test
     * ở trên - đảm bảo màn Boot có ĐỦ nội dung trước khi Task_Display nhận
     * bit và có thể rời màn hình (task_display.c còn tự đảm bảo thời gian
     * hiển thị tối thiểu, xem comment trong file đó). */
    osEventFlagsSet(SystemEventGroupHandle, EVT_BIT_BOOT_DONE);

    /* THÊM: khôi phục ĐÚNG timeout IWDG gốc (~0.5s) ngay trước khi vào vòng
     * lặp chính - không để hệ thống chạy lâu dài với watchdog bị nới lỏng
     * (giữ đúng độ nhạy bảo vệ ban đầu cho toàn bộ control loop 100Hz). */
    calibration_data_iwdg_restore_orig();

    /* Giá trị không hợp lệ ban đầu để ép gọi *_enter() ngay lần đầu vào
     * 1 mode bất kỳ (kể cả khi mode đầu tiên đúng bằng 0/OPMODE_HOME). */
    uint8_t s_last_mode = 0xFFu;

    for (;;) {
        //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);

        system_state_t state = system_state_get();

        if (state != STATE_RUN) {
            /* Giữ đúng hành vi cũ: không dispatch mode gì khi không RUN,
             * chỉ step() để servo dừng êm tại vị trí hiện tại (không giật
             * khi chuyển STOP/READY) - servo_actuator_step() không đổi
             * target nếu không ai gọi set_target()/apply_delta() trước đó. */
            servo_actuator_step(CONTROL_LOOP_DT_S);
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
            osDelay(CONTROL_LOOP_PERIOD_MS);
            continue;
        }

        setpoint_t sp;
        if (!setpoint_get(&sp)) {
            /* Không lấy được mutex trong 5 tick (setpoint_get() timeout nội
             * bộ) - bỏ qua chu kỳ này, KHÔNG đổi target theo dữ liệu rác,
             * chỉ step() giữ nguyên vị trí hiện tại. */
            servo_actuator_step(CONTROL_LOOP_DT_S);
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
            osDelay(CONTROL_LOOP_PERIOD_MS);
            continue;
        }

        bool mode_changed = (sp.mode != s_last_mode);

        switch (sp.mode) {
            case OPMODE_HOME:
                if (mode_changed) {
                    control_mode_home_enter();
                }
                control_mode_home_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_MANUAL:
                if (mode_changed) {
                    control_mode_manual_enter();
                }
                control_mode_manual_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_CALIB:
                if (mode_changed) {
                    control_mode_calib_enter();
                }
                control_mode_calib_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_BALANCE:
                if (mode_changed) {
                    control_mode_balance_enter();
                }
                control_mode_balance_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_POSITION:
                if (mode_changed) {
                    control_mode_position_enter();
                }
                control_mode_position_step(CONTROL_LOOP_DT_S);
                break;

            /* TODO Giai đoạn 5: case OPMODE_BALANCE   -> control_mode_balance_enter/step() */
            /* TODO Giai đoạn 5: case OPMODE_POSITION  -> control_mode_position_enter/step() */

            default:
                /* Mode chưa cài code (Calib/Balance/Position/Manual ở Giai
                 * đoạn 2) - KHÔNG gọi set_target/apply_delta gì cả, servo
                 * giữ nguyên vị trí hiện tại (an toàn), tránh hành vi không
                 * xác định nếu UI lỡ chọn mode chưa có code thật. */
                break;
        }

        s_last_mode = sp.mode;

        servo_actuator_step(CONTROL_LOOP_DT_S);   /* luôn gọi cuối cùng, 1 lần/chu kỳ */
        task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}

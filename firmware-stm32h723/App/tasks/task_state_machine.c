#include "task_state_machine.h"
#include "system_state.h"
#include "cmsis_os2.h"
#include "main.h"

extern osMessageQueueId_t StateRequestQueueHandle;   // kiểm tra đúng tên CubeMX sinh trong freertos.c

void StartStateMachine(void *argument)
{
    state_event_t evt;
    system_state_publish(STATE_INIT);   // entry action INIT

    /* =========================================================
     * TEMP BRING-UP CHO B5 - XOÁ KHỐI NÀY khi có Task_Safety/
     * Task_Calibration THẬT tự gửi EVT_SELFTEST_OK/EVT_CALIB_DONE
     * qua StateRequestQueueHandle.
     * Lý do cần tạm thời: nếu không có, hệ đứng yên ở STATE_INIT
     * vô thời hạn (không ai gửi 2 event trên) -> không bao giờ tới
     * được STATE_READY -> BTN1-long trong Task_Button_UI không có
     * tác dụng gì (không phải bug ở đó, chỉ là thiếu bước này).
     * ========================================================= */
    system_state_publish(STATE_CALIBRATION);
    system_state_publish(STATE_READY);
    /* ========================= HẾT TEMP ========================= */

    for (;;) {
        if (osMessageQueueGet(StateRequestQueueHandle, &evt, NULL, osWaitForever) == osOK) {
            system_state_t cur = system_state_get();

            if (evt == EVT_FAULT_DETECTED) {
                system_state_publish(STATE_ERROR);
                /* SUA (muc 2): chuyen thang sang SAFE_MODE ngay tai day, KHONG
                 * cho event tiep theo moi chay toi case STATE_ERROR ben duoi.
                 * Bug cu: neu sau EVT_FAULT_DETECTED khong ai gui them event
                 * nao vao StateRequestQueueHandle, osMessageQueueGet() o dau
                 * vong lap se chan osWaitForever -> case STATE_ERROR khong
                 * bao gio duoc switch toi -> he ket vinh vien o STATE_ERROR. */
                system_state_publish(STATE_SAFE_MODE);
                continue;
            }
            switch (cur) {
                case STATE_INIT:
                    if (evt == EVT_SELFTEST_FAIL) system_state_publish(STATE_ERROR);
                    else if (evt == EVT_CALIB_DONE) system_state_publish(STATE_READY);
                    else if (evt == EVT_SELFTEST_OK) system_state_publish(STATE_CALIBRATION);
                    break;
                case STATE_CALIBRATION:
                    if (evt == EVT_CALIB_DONE) system_state_publish(STATE_READY);
                    else if (evt == EVT_CALIB_FAIL) system_state_publish(STATE_ERROR);
                    break;
                case STATE_READY:
                    if (evt == EVT_BTN_RUN) system_state_publish(STATE_RUN);
                    else if (evt == EVT_BTN_SLEEP) system_state_publish(STATE_SLEEP);
                    break;
                case STATE_RUN:
                    if (evt == EVT_BTN_STOP) system_state_publish(STATE_READY);
                    /* ---- THÊM cho B6 ----
                     * Đây là nơi DUY NHẤT được publish state khi Home/Calib
                     * xong - đúng nguyên tắc "chỉ Task_StateMachine ghi
                     * system_state". control_mode_home.c/control_mode_calib.c
                     * KHÔNG được gọi system_state_publish() trực tiếp, chỉ
                     * được gửi 3 event này qua StateRequestQueueHandle. */
                    else if (evt == EVT_HOME_DONE) system_state_publish(STATE_READY);
                    else if (evt == EVT_MODE_CALIB_DONE) system_state_publish(STATE_READY);
                    else if (evt == EVT_MODE_CALIB_FAILED) system_state_publish(STATE_ERROR);
                    /* EVT_MODE_CALIB_FAILED -> STATE_ERROR: không auto-forward
                     * sang SAFE_MODE ngay tại đây (khác EVT_FAULT_DETECTED ở
                     * trên) - để giữ đúng hành vi case STATE_ERROR bên dưới
                     * (tự chuyển SAFE_MODE khi có event tiếp theo tới, đã
                     * sửa ở B5 mục 2). Calib lỗi là lỗi cơ khí/hội tụ, không
                     * phải fault an toàn khẩn cấp như EVT_FAULT_DETECTED,
                     * nên không cần bắt buộc phản ứng tức thì trong 1 dòng. */
                    break;
                case STATE_SLEEP:
                    if (evt == EVT_BTN_WAKE) system_state_publish(STATE_INIT);
                    break;
                case STATE_ERROR:
                    /* Lop bao ve du phong: neu co duong nao khac lam
                     * system_state_publish(STATE_ERROR) truc tiep (khong qua
                     * EVT_FAULT_DETECTED o tren), van tu chuyen tiep sang
                     * SAFE_MODE khi co event moi toi, giu nguyen hanh vi cu. */
                    system_state_publish(STATE_SAFE_MODE);
                    break;
                case STATE_SAFE_MODE:
                    if (evt == EVT_MANUAL_RESET_ACK) system_state_publish(STATE_INIT);
                    break;
                default: break;
            }
        }
    }
}

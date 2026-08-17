#ifndef TASK_STATE_MACHINE_H
#define TASK_STATE_MACHINE_H

typedef enum {
    EVT_SELFTEST_OK, EVT_SELFTEST_FAIL, EVT_CALIB_DONE, EVT_CALIB_FAIL,
    EVT_BTN_RUN, EVT_BTN_STOP, EVT_BTN_SLEEP, EVT_BTN_WAKE,
    EVT_FAULT_DETECTED, EVT_MANUAL_RESET_ACK,

    /* ---- THÊM cho B6 (control_mode_home.c / control_mode_calib.c) ----
     * Đặt tên RIÊNG với EVT_CALIB_DONE/EVT_CALIB_FAIL (2 event đó dùng cho
     * luồng self-test bring-up ở STATE_INIT/STATE_CALIBRATION, mục 3 trong
     * task_state_machine.c) để tránh 1 tên event mang 2 ý nghĩa khác nhau ở
     * 2 ngữ cảnh khác nhau (dễ gây nhầm lẫn khi đọc code/log sau này).
     * Cả 3 event dưới đây chỉ được xử lý khi cur == STATE_RUN (xem .c). */
    EVT_HOME_DONE,          // control_mode_home: đã về neutral ổn định -> RUN->READY
    EVT_MODE_CALIB_DONE,    // control_mode_calib: P-controller hội tụ + đã lưu Flash -> RUN->READY
    EVT_MODE_CALIB_FAILED,  // control_mode_calib: vượt MAX_ITER, không hội tụ -> RUN->ERROR
} state_event_t;

typedef enum {
    STATE_BOOT, STATE_INIT, STATE_CALIBRATION, STATE_READY,
    STATE_RUN, STATE_SLEEP, STATE_ERROR, STATE_SAFE_MODE
} system_state_t;

void StartStateMachine(void *argument);   // entry function do CubeMX sinh, khai báo lại đây cho rõ

#endif

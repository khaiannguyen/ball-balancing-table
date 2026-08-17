#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"

#include "task_button_ui.h"
#include "buttons.h"
#include "screen.h"
#include "screen_gauge_common.h"   /* THAY screen_stop.h - dùng ScreenGauge_SetStopped()
                                      thay vì chuyển sang ScreenStop_Get() riêng */
#include "system_state.h"
#include "task_state_machine.h"   /* state_event_t, system_state_t */
#include "main.h"

/* CubeMX sinh trong freertos.c (đúng như task_state_machine.c đã extern) -
 * kiểm tra lại tên khớp CubeMX của bạn nếu build báo undefined reference. */
extern osMessageQueueId_t StateRequestQueueHandle;

/* =========================================================
 * B5 - Task_Button_UI, dùng ĐÚNG cơ chế thật của Task_StateMachine
 * (task_state_machine.c đã có sẵn, KHÔNG còn hack tạm publish thẳng
 * nữa). Task_Button_UI CHỈ gửi state_event_t qua StateRequestQueueHandle -
 * Task_StateMachine là nơi DUY NHẤT gọi system_state_publish(), đúng
 * nguyên tắc sole-writer đã ghi trong system_state.h.
 *
 * Ánh xạ:
 *   BTN_ID_1 short -> BUTTON_ENTER (qua ScreenManager)
 *   BTN_ID_1 long  -> gửi EVT_BTN_RUN/EVT_BTN_STOP tuỳ state hiện tại,
 *                     HOẶC thoát SAFE_MODE nếu đang kẹt ở đó (THÊM, xem dưới)
 *   BTN_ID_2..6    -> UP/LEFT/DOWN/RIGHT/EXIT (qua ScreenManager)
 * ========================================================= */

static void HandleBtn1Long(void)
{
    system_state_t cur = system_state_get();
    bool inFault = system_state_fault_is_set();

    if (cur == STATE_RUN)
    {
        state_event_t evt = EVT_BTN_STOP;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        /* Không vẽ overlay nếu đang hiện Fault - Task_Display đã
         * GotoAndRemember(ScreenFault) và chiếm hẳn màn hình, vẽ đè STOP
         * lên đó không có ý nghĩa (và sẽ bị Fault screen ghi đè lại ở
         * update() 25Hz kế tiếp). stopOverlayActive vẫn được set true
         * trong screen_gauge_common.c ở lần EnterForMode() kế tiếp khi
         * Fault hết và GoBack() quay lại gauge screen (xem comment
         * trong EnterForMode()). */
        if (!inFault)
        {
            ScreenGauge_SetStopped(true);   /* THAY GotoAndRemember(ScreenStop_Get()) */
        }
    }
    else if (cur == STATE_READY)
    {
        state_event_t evt = EVT_BTN_RUN;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        if (!inFault)
        {
            ScreenGauge_SetStopped(false);  /* THAY ScreenManager_GoBack() */
        }
    }
    /* THÊM: thoát STATE_SAFE_MODE bằng BTN1 long - CHỈ khi fault THẬT SỰ
     * đã hết (system_state_fault_is_set() == false). Đây là bước xác nhận
     * thủ công bắt buộc (đúng tinh thần thiết kế "SAFE_MODE không tự
     * thoát" trong task_can_rx.c) - an toàn hơn thoát vô điều kiện: nếu
     * CAN vẫn đang mất kết nối thật (inFault vẫn true), bấm nút không có
     * tác dụng gì, tránh nhảy ra rồi lập tức bị đẩy lại vào SAFE_MODE ngay
     * chu kỳ sau (gây nhấp nháy state khó hiểu cho người dùng).
     *
     * TẠM THỜI (bring-up, giống tinh thần đoạn TEMP trong
     * task_state_machine.c): vì hiện tại CHƯA có Task_Safety/
     * Task_Calibration thật tự gửi EVT_SELFTEST_OK/EVT_CALIB_DONE, sau khi
     * EVT_MANUAL_RESET_ACK đưa hệ về STATE_INIT, phải TỰ gửi tiếp 2 event
     * đó ngay tại đây để không bị kẹt vĩnh viễn ở STATE_INIT. KHI CÓ
     * Task_Safety/Task_Calibration THẬT: xoá đoạn gửi
     * EVT_SELFTEST_OK/EVT_CALIB_DONE ở đây (giữ lại
     * EVT_MANUAL_RESET_ACK), để state machine đi qua đúng luồng
     * INIT -> CALIBRATION -> READY bằng self-test/calib thật, không phải
     * giả lập ở nút bấm. */
    else if (cur == STATE_SAFE_MODE && !inFault)
    {
        state_event_t evt;

        evt = EVT_MANUAL_RESET_ACK;
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        evt = EVT_SELFTEST_OK;   /* TODO: xoá khi có Task_Safety thật */
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);

        evt = EVT_CALIB_DONE;    /* TODO: xoá khi có Task_Calibration thật */
        osMessageQueuePut(StateRequestQueueHandle, &evt, 0, 0);
    }
    /* else: đang ở INIT/CALIBRATION/SLEEP/ERROR, hoặc SAFE_MODE nhưng
     * fault vẫn còn thật - BTN1 long không có ý nghĩa, không gửi gì. */
}

void StartTaskButtonUi(void *argument)
{
    (void)argument;

    button_event_t evt;

    for (;;)
    {
        osStatus_t st = osMessageQueueGet(ButtonEventQueueHandle, &evt, NULL, osWaitForever);

        if (st == osOK)
        {
            ButtonState_t s;
            s.event = BUTTON_EVENT_PRESS;
            bool forward = true;

            switch (evt.id)
            {
            case BTN_ID_1:
                if (evt.type == BTN_EVT_SHORT_PRESS)
                {
                    s.button = BUTTON_ENTER;
                    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
                }
                else if (evt.type == BTN_EVT_LONG_PRESS)
                {
                    HandleBtn1Long();
                    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
                    forward = false;
                }
                else
                {
                    forward = false;
                }
                break;

            case BTN_ID_2: s.button = BUTTON_UP;    break;
            case BTN_ID_3: s.button = BUTTON_LEFT;  break;
            case BTN_ID_4: s.button = BUTTON_DOWN;  break;
            case BTN_ID_5: s.button = BUTTON_RIGHT; break;
            case BTN_ID_6: s.button = BUTTON_EXIT;  break;

            default:
                forward = false;
                break;
            }

            if (forward)
            {
                ScreenManager_OnButton(s);
            }
        }

        /* Task_Button_UI KHÔNG nằm trong ALIVE_MASK_EXPECTED (mục 3.6 chỉ
         * định nghĩa bit cho ControlLoop/IMU_Fusion/CAN_RX) - giống lý do
         * task_can_tx.c không gọi task_alive_mark(), task này cũng KHÔNG
         * được gọi. */
    }
}

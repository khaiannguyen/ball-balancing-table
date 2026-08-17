#include "control_mode_manual.h"
#include "servo_test.h"
#include "ui_data.h"
#include <string.h>

static manual_sub_state_t s_sub          = MANUAL_SUB_IDLE;
static bool                s_sub_entered  = false;

static void set_guide(const char *msg)
{
    strncpy(g_uiData.guideText, msg, sizeof(g_uiData.guideText) - 1);
    g_uiData.guideText[sizeof(g_uiData.guideText) - 1] = '\0';
}

void control_mode_manual_enter(void)
{
    servo_test_stop();
    s_sub         = MANUAL_SUB_IDLE;
    s_sub_entered = false;
    set_guide("Manual: UP/DOWN to select");
}

void control_mode_manual_step(float dt)
{
    switch (s_sub) {
        case MANUAL_SUB_IDLE:
            /* Chờ chọn qua control_mode_manual_select_substate() từ UI */
            break;

        case MANUAL_SUB_MANUAL_STEP:
            if (!s_sub_entered) {
                servo_test_start(SERVO_TEST_MODE_MANUAL_STEP, 1 /* mặc định S1 */);
                s_sub_entered = true;
                set_guide("Manual: LEFT/RIGHT adjust us");
            }
            servo_test_step_dt(dt);
            /* Không tự "done" - chờ người dùng đổi sub-state khác qua UI */
            break;

        case MANUAL_SUB_SWEEP_LOG:
            if (!s_sub_entered) {
                servo_test_start(SERVO_TEST_MODE_SWEEP_LOG, 0 /* cả 3 servo */);
                s_sub_entered = true;
                set_guide("Manual: sweep log CSV...");
            }
            servo_test_step_dt(dt);
            if (servo_test_is_done()) {
                s_sub         = MANUAL_SUB_DONE;
                s_sub_entered = false;
                set_guide("Sweep done. Log UART");
            }
            break;

        case MANUAL_SUB_DONE:
            /* Chờ người dùng chọn việc khác hoặc thoát Mode Manual (đổi mode
             * khác trên UI - control_mode_manual_enter() sẽ reset lại). */
            break;
    }
}

manual_sub_state_t control_mode_manual_get_sub_state(void)
{
    return s_sub;
}

void control_mode_manual_select_substate(manual_sub_state_t sub)
{
    servo_test_stop();
    s_sub         = sub;
    s_sub_entered = false;
}

void control_mode_manual_adjust(int16_t delta_us)
{
    if (s_sub == MANUAL_SUB_MANUAL_STEP) {
        servo_test_manual_adjust(delta_us);
    }
}

void control_mode_manual_select_channel(uint8_t servo_ch)
{
    if (s_sub == MANUAL_SUB_MANUAL_STEP) {
        servo_test_manual_select_channel(servo_ch);
    }
}

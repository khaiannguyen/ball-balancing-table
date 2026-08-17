#ifndef TASK_CONTROL_LOOP_H
#define TASK_CONTROL_LOOP_H

/**
 * @file  task_control_loop.h
 * @brief Task chính điều khiển servo - dispatch theo setpoint.mode
 *        (operating_mode_t, system_state.h) khi system_state == STATE_RUN.
 *        Giai đoạn 2 (B6): chỉ OPMODE_HOME có code thật, các mode khác
 *        (Calib/Balance/Position/Manual) giữ servo đứng yên tại chỗ, sẽ
 *        nối dần ở các giai đoạn sau (xem B6_Control.md mục 6).
 */

/* entry function do CubeMX sinh (osThreadNew trong main.c) - khai báo lại
 * đây cho rõ, cùng pattern với StartStateMachine (task_state_machine.h). */
void StartTaskControlLoop(void *argument);

#endif /* TASK_CONTROL_LOOP_H */

/**
 * @file    task_control_loop.c
 * @brief   Main real-time control task.
 *
 * Initializes the control subsystem during boot and executes the active
 * control mode at a fixed 100 Hz rate while the system is in STATE_RUN.
 *
 * The task owns control-mode dispatching and always advances the actuator
 * trajectory once per control cycle.
 */

#include "task_control_loop.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>

#include "imu_mpu6500.h"
#include "buttons.h"
#include "servo_actuator.h"
#include "system_state.h"
#include "calibration_data.h"

#include "control_mode_home.h"
#include "control_mode_manual.h"
#include "control_mode_calib.h"
#include "control_mode_balance.h"
#include "control_mode_position.h"

#include "screen_boot.h"

extern SPI_HandleTypeDef hspi2;

extern osEventFlagsId_t SystemEventGroupHandle;
extern osMessageQueueId_t StateRequestQueueHandle;

#define CONTROL_LOOP_DT_S       0.01f
#define CONTROL_LOOP_PERIOD_MS  10

void StartTaskControlLoop(void *argument)
{
    (void)argument;

    /*
     * Temporarily widen the independent watchdog during boot initialization.
     *
     * Control-loop initialization includes TFT synchronization, IMU setup,
     * calibration loading, servo initialization, and boot-screen logging.
     * These operations can take longer than the normal runtime watchdog window.
     *
     * The original watchdog configuration is restored immediately before
     * entering the normal 100 Hz control loop.
     */
    calibration_data_iwdg_widen_for_boot();

    /*
     * Wait for the display task to complete TFT initialization before sending
     * boot-screen log messages.
     *
     * The wait is bounded so a display initialization failure cannot prevent
     * the control task from continuing with IMU, calibration, and actuator
     * initialization.
     *
     * The wait is split into short intervals so the control task continues
     * reporting liveness while waiting for the display-ready event.
     */
    #define TFT_READY_POLL_MS       200u
    #define TFT_READY_TOTAL_MS      2000u

    {
        uint32_t poll_ticks =
            (TFT_READY_POLL_MS * osKernelGetTickFreq()) / 1000u;

        uint32_t waited_ms = 0u;
        uint32_t flags;

        do
        {
            flags =
                osEventFlagsWait(
                    SystemEventGroupHandle,
                    EVT_BIT_TFT_READY,
                    osFlagsWaitAny,
                    poll_ticks
                );

            /*
             * Keep the watchdog alive while boot synchronization is in
             * progress. A successful event exits the wait immediately.
             */
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

            if ((flags & osFlagsError) == 0)
                break;

            waited_ms += TFT_READY_POLL_MS;

        } while (waited_ms < TFT_READY_TOTAL_MS);
    }

    /*
     * Initialize the IMU before entering the runtime control loop.
     *
     * The result is retained for the boot self-test and state-machine
     * transition below.
     */
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

    bool imu_ok = imu_mpu6500_init(&hspi2);

    printf(
        "MPU6500 init: %s\r\n",
        imu_ok ? "OK" : "FAIL"
    );

    ScreenBoot_AddLog(
        imu_ok ? "IMU MPU6500: OK" : "IMU MPU6500: FAIL"
    );

    /*
     * Keep the watchdog alive after potentially blocking boot-screen output.
     */
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /*
     * Initialize the button input service before the system enters its
     * normal interactive states.
     */
    buttons_init();

    ScreenBoot_AddLog("Buttons: OK");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /*
     * Load persistent calibration data into RAM.
     *
     * If no valid calibration is stored, the calibration module provides
     * its defined safe defaults. Calibration validity is checked separately
     * before allowing the system to proceed to normal operation.
     */
    bool calib_load_ok = calibration_data_load();

    ScreenBoot_AddLog(
        calib_load_ok
            ? "Calib data: LOADED"
            : "Calib data: DEFAULT (chua calib)"
    );

    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /*
     * Apply the loaded calibration to the actuator layer before actuator
     * initialization so the initial position and target values use the
     * calibrated neutral points.
     *
     * This keeps calibration data ownership centralized in the calibration
     * module instead of duplicating calibration writes in this task.
     */
    calibration_data_apply_to_actuator();

    ScreenBoot_AddLog("Calib -> Servo: applied");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /*
     * Initialize the actuator layer after calibration has been applied.
     *
     * Manual servo test behavior is owned by control_mode_manual and is not
     * executed automatically by the control task during initialization.
     */
    servo_actuator_init();

    ScreenBoot_AddLog("Servo actuator: OK");
    task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

    /*
     * Publish boot self-test results through the state-request queue.
     *
     * A failed IMU initialization prevents normal operation. Calibration
     * validity is reported separately so the state machine can require a
     * valid calibration before entering modes that depend on calibrated
     * actuator geometry.
     */
    {
        state_event_t evt;

        if (imu_ok)
        {
            evt = EVT_SELFTEST_OK;

            osMessageQueuePut(
                StateRequestQueueHandle,
                &evt,
                0,
                0
            );

            ScreenBoot_AddLog("Self-test: OK");
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

            bool calib_valid = calibration_data_is_valid();

            evt =
                calib_valid
                    ? EVT_CALIB_DONE
                    : EVT_CALIB_FAIL;

            osMessageQueuePut(
                StateRequestQueueHandle,
                &evt,
                0,
                0
            );

            ScreenBoot_AddLog(
                calib_valid
                    ? "Calib: VALID"
                    : "Calib: INVALID - can calib lai!"
            );

            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
        }
        else
        {
            evt = EVT_SELFTEST_FAIL;

            osMessageQueuePut(
                StateRequestQueueHandle,
                &evt,
                0,
                0
            );

            ScreenBoot_AddLog("Self-test: FAIL (IMU)");
            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);
        }

        /*
         * Calibration validity is determined by calibration_data_is_valid().
         * Keep the load result available for diagnostics without using it as
         * the runtime source of truth.
         */
        (void)calib_load_ok;
    }

    ScreenBoot_AddLog("READY");

    /*
     * Signal that all control-loop initialization steps are complete.
     *
     * The display task can use this event to finish the boot-screen sequence
     * without relying on a fixed delay.
     */
    osEventFlagsSet(
        SystemEventGroupHandle,
        EVT_BIT_BOOT_DONE
    );

    /*
     * Restore the normal watchdog timeout before entering runtime operation.
     * The extended boot timeout must never remain active during control.
     */
    calibration_data_iwdg_restore_orig();

    /*
     * Force the first control cycle to execute the selected mode's enter()
     * function even when the initial mode value is OPMODE_HOME (zero).
     */
    uint8_t s_last_mode = 0xFFu;

    for (;;)
    {
        system_state_t state = system_state_get();

        /*
         * Control modes are executed only in STATE_RUN.
         *
         * Outside RUN, continue advancing the actuator trajectory so the
         * servos can settle smoothly at their current targets without
         * introducing an abrupt position change.
         */
        if (state != STATE_RUN)
        {
            servo_actuator_step(CONTROL_LOOP_DT_S);

            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

            osDelay(CONTROL_LOOP_PERIOD_MS);

            continue;
        }

        setpoint_t sp;

        /*
         * Do not execute a control cycle with an invalid or unavailable
         * setpoint. Keep the current actuator target and allow the next
         * cycle to retry the read.
         */
        if (!setpoint_get(&sp))
        {
            servo_actuator_step(CONTROL_LOOP_DT_S);

            task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

            osDelay(CONTROL_LOOP_PERIOD_MS);

            continue;
        }

        bool mode_changed =
            (sp.mode != s_last_mode);

        /*
         * Enter a control mode only when the selected mode changes.
         * The step function then executes once per 10 ms control cycle.
         */
        switch (sp.mode)
        {
            case OPMODE_HOME:
                if (mode_changed)
                {
                    control_mode_home_enter();
                }

                control_mode_home_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_MANUAL:
                if (mode_changed)
                {
                    control_mode_manual_enter();
                }

                control_mode_manual_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_CALIB:
                if (mode_changed)
                {
                    control_mode_calib_enter();
                }

                control_mode_calib_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_BALANCE:
                if (mode_changed)
                {
                    control_mode_balance_enter();
                }

                control_mode_balance_step(CONTROL_LOOP_DT_S);
                break;

            case OPMODE_POSITION:
                if (mode_changed)
                {
                    control_mode_position_enter();
                }

                control_mode_position_step(CONTROL_LOOP_DT_S);
                break;

            default:
                /*
                 * Unknown or unsupported modes must not modify actuator
                 * targets. The actuator layer continues from its current
                 * target, providing a safe fallback for invalid requests.
                 */
                break;
        }

        s_last_mode = sp.mode;

        /*
         * Advance the actuator trajectory exactly once at the end of every
         * control cycle, regardless of the selected control mode.
         */
        servo_actuator_step(CONTROL_LOOP_DT_S);

        task_alive_mark(ALIVE_BIT_CONTROL_LOOP);

        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}

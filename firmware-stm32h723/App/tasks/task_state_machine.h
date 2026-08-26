/**
 * @file    task_state_machine.h
 * @brief   System state-machine interface.
 *
 * Defines the events and top-level runtime states used by the system
 * state machine.
 *
 * State transitions are owned by Task_StateMachine. Other application
 * tasks request transitions by sending events through the state queue.
 */

#ifndef TASK_STATE_MACHINE_H
#define TASK_STATE_MACHINE_H

/**
 * @brief Events consumed by the system state machine.
 *
 * Events represent asynchronous requests or results that may cause a
 * transition between system states.
 */
typedef enum
{
    EVT_SELFTEST_OK,
    EVT_SELFTEST_FAIL,

    EVT_CALIB_DONE,
    EVT_CALIB_FAIL,

    EVT_BTN_RUN,
    EVT_BTN_STOP,
    EVT_BTN_SLEEP,
    EVT_BTN_WAKE,

    EVT_FAULT_DETECTED,
    EVT_MANUAL_RESET_ACK,

    /*
     * Runtime control-mode completion events.
     *
     * These events allow control modes to report completion without
     * modifying the global system state directly.
     */
    EVT_HOME_DONE,
    EVT_MODE_CALIB_DONE,
    EVT_MODE_CALIB_FAILED

} state_event_t;

/**
 * @brief Top-level system states.
 *
 * Defines the states used by the system-level state machine to control
 * initialization, normal operation, sleep, and fault handling.
 */
typedef enum
{
    STATE_BOOT,
    STATE_INIT,
    STATE_CALIBRATION,
    STATE_READY,
    STATE_RUN,
    STATE_SLEEP,
    STATE_ERROR,
    STATE_SAFE_MODE

} system_state_t;

/**
 * @brief Run the system state-machine task.
 *
 * Consumes state events and performs the corresponding top-level system
 * state transitions.
 *
 * @param argument FreeRTOS task argument. Not used.
 */
void StartStateMachine(void *argument);

#endif /* TASK_STATE_MACHINE_H */

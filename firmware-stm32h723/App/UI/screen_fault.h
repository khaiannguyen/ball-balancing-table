#ifndef SCREEN_FAULT_H
#define SCREEN_FAULT_H

#include "screen.h"

/*
 * @brief Screen shown while the system is in a fault state.
 *
 * Displayed whenever system_state_fault_is_set() returns true, polled
 * by Task_Display on every loop iteration. Returns automatically to
 * whichever screen was active beforehand once the fault clears - see
 * Task_Display in task_display.c. No button press is required to
 * exit this screen.
 */
const Screen_t* ScreenFault_Get(void);

#endif

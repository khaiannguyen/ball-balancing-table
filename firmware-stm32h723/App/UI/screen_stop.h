#ifndef SCREEN_STOP_H
#define SCREEN_STOP_H

#include "screen.h"

/* Static STOP screen displayed after a BTN1 long-press
 * switches the robot to STOP mode.
 *
 * The screen intentionally does not handle button events.
 * Exiting STOP requires another BTN1 long-press, which is
 * handled directly by Task_Button_UI rather than the
 * ScreenManager button dispatch path.
 */
const Screen_t* ScreenStop_Get(void);

#endif

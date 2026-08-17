#ifndef SCREEN_SHUTDOWN_H
#define SCREEN_SHUTDOWN_H
#include "screen.h"
const Screen_t* ScreenShutdown_Get(void);
void ScreenShutdown_AddLog(const char *line);
#endif

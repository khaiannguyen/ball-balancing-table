#ifndef SCREEN_BOOT_H
#define SCREEN_BOOT_H
#include "screen.h"
const Screen_t* ScreenBoot_Get(void);
void ScreenBoot_AddLog(const char *line);
void ScreenBoot_ClearLog(void);
#endif

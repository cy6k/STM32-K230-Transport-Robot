#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"

//按键1：PA4；按键2：PB1；下拉输入，按键另一端接 3.3V，高电平按下
void Key_Init(void);

uint8_t Key1_get();
uint8_t Key2_get();

#endif

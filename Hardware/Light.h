#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx.h"

// PA4 普通输出，初始化后默认关灯。
void Light_Init(void);

// 使用前先调用 Light_Init()。
void Light_on(void);
void Light_off(void);

#endif


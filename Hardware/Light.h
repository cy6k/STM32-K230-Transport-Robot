#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx.h"

// PA4 软件 PWM，占用 TIM7；默认关闭，标称频率 1kHz
void Light_Init(void);

// Brightness：0~100 对应占空比百分数，超过 100 按 100 处理
void Light_set(uint8_t Brightness);

// State：0 关闭，非 0 全亮
void Light_Control(uint8_t State);

#endif

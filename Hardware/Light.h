#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx.h"

// PB0 硬件 PWM（TIM3_CH3，AF2），独占 TIM3；默认关闭，频率 20kHz
void Light_Init(void);

// Brightness：0~100 对应占空比百分数，超过 100 按 100 处理
void Light_set(uint8_t Brightness);

// 关闭补光灯。
void Light_off(void);

#endif

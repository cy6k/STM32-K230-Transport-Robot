#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx.h"

// PB0 硬件 PWM（TIM3_CH3，AF2），独占 TIM3；默认关闭，频率 20kHz
void Light_Init(void);

// Brightness：0~100 对应占空比百分数，超过 100 按 100 处理
void Light_set(uint8_t Brightness);

// State：0 关闭，非 0 全亮
void Light_Control(uint8_t State);

// 原开关接口：全亮/关闭。
void Light_on(void);
void Light_off(void);

#endif

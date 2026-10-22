/**
  ******************************************************************************
  * @file    Timer.h
  * @brief   非阻塞定时模块
  * @note    TIM6 每 1ms 中断一次; GetTick() 返回毫秒计数(非阻塞)
  ******************************************************************************
  */
#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f4xx.h"

// TIM6 保持 1ms 中断；此计数每 200ms 加 1，应用层只读取，不清零。
extern volatile uint32_t Timer_200ms_Count;

void     Timer_Init(void);   //启动 1ms 定时器
uint32_t GetTick(void);      //返回系统毫秒数(自Timer_Init后)

#endif

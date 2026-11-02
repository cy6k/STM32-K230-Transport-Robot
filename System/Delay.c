#include "stm32f4xx.h"

/**
  * @brief  微秒级延时(阻塞式, 用 SysTick)
  * @param  xus 延时时长, 范围: 0~99864 (168MHz 下单次最大约 1ms)
  * @retval 无
  * @note   SysTick 时钟源 = HCLK = 168MHz, 所以 1us = 168 个计数
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = SystemCoreClock / 1000000 * xus - 1;	//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长, 范围: 0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}


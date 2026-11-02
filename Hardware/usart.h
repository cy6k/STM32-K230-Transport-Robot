#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"
#include "stdbool.h"

/**********************************************************
***	张大头 Emm_V5.0 通讯串口 —— 本板移植版
***
***	官方例程用的是 USART1(PA9/PA10)，本板张大头电机口接的是
***	USART2(PA2/PA3)，所以这里改成 USART2，仅发送控制指令，不接收反馈。
***
***	本板接线：CN1 / CN2 的
***	  1脚 = VIN(电源正 7-32V)   -> 电机 V+
***	  2脚 = GND                 -> 电机 Gnd
***	  3脚 = PA2 (MCU_TX)        -> 电机 R/A/H
***	  4脚 = PA3 (MCU_RX)        -> 电机 T/B/L
***	CN1 与 CN2 并联在同一条 USART2 总线上，两台电机地址必须不同。
**********************************************************/

void usart_init(void);								// 初始化 USART2 + PA2，仅发送
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);	// 发送多个字节(Emm_V5.c 唯一依赖)
void usart_SendByte(uint16_t data);					// 发送一个字节

/* 串口屏收发与按键队列，与电机串口共用本模块。 */
extern volatile uint32_t tjc_key_dropped;
void TJC_USART6_Init(void);
uint8_t TJC_GetStartKey(char *side, uint8_t *mode);

#endif

#include "usart.h"

/**********************************************************
***	张大头 Emm_V5.0 通讯串口 —— 本板移植版 (USART2, PA2/PA3)
***	原例程为 USART1(PA9/PA10)，仅把外设与引脚换成 USART2，
***	收发逻辑与官方完全一致：
***	  接收 = RXNE 中断逐字节入 FIFO，IDLE 中断判定一帧结束
***	  发送 = 轮询 TXE，带 8000 次超时保护，不会死等
**********************************************************/

__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

/**
	* @brief   初始化USART2 (PA2 = TX, PA3 = RX)
	* @param   无
	* @retval  无
	* @note    NVIC 优先级分组由 main 里的 NVIC_PriorityGroupConfig 决定，
	*          本工程是 NVIC_PriorityGroup_2，这里用抢占1/子0，
	*          低于 TIM6 的抢占0，保证 1ms 节拍不被串口打断。
	*/
void usart_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

/**********************************************************
***	使能时钟：GPIOA 在 AHB1，USART2 在 APB1
**********************************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

/**********************************************************
***	初始化USART2引脚：PA2 - USART2_TX、PA3 - USART2_RX
**********************************************************/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;			/* 复用推挽输出 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;			/* 上拉，空闲高电平 */
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 将PA2、PA3复用为USART2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

/**********************************************************
***	初始化USART2：115200-8-N-1，与电机菜单 UartBaud 一致
**********************************************************/
	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

/**********************************************************
***	配置USART2中断
**********************************************************/
	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/**********************************************************
***	清除USART2中断
**********************************************************/
	USART2->SR; USART2->DR;
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);

/**********************************************************
***	使能USART2中断
**********************************************************/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

/**********************************************************
***	初始化接收队列并使能USART2
**********************************************************/
	initQueue();
	USART_Cmd(USART2, ENABLE);
}

/**
	* @brief   USART2中断函数
	* @param   无
	* @retval  无
	*/
void USART2_IRQHandler(void)
{
	__IO uint16_t i = 0;

/**********************************************************
***	串口接收中断
**********************************************************/
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		// 未完成一帧数据接收，数据进入缓冲队列
		fifo_enQueue((uint8_t)USART2->DR);

		// 清除串口接收中断
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}

/**********************************************************
***	串口空闲中断
**********************************************************/
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		// 先读SR再读DR，清除IDLE中断
		USART2->SR; USART2->DR;

		// 提取一帧数据命令
		rxCount = fifo_queueLength(); for(i=0; i < rxCount; i++) { rxCmd[i] = fifo_deQueue(); }

		// 一帧数据接收完成，置位帧标志位
		rxFrameFlag = true;
	}
}

/**
	* @brief   USART发送多个字节
	* @param   无
	* @retval  无
	*/
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0;

	for(i=0; i < len; i++) { usart_SendByte(cmd[i]); }
}

/**
	* @brief   USART发送一个字节
	* @param   无
	* @retval  无
	*/
void usart_SendByte(uint16_t data)
{
	__IO uint16_t t0 = 0;

	USART2->DR = (data & (uint16_t)0x01FF);

	while(!(USART2->SR & USART_FLAG_TXE))
	{
		++t0; if(t0 > 8000)	{	return; }
	}
}

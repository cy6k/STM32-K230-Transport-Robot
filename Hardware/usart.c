#include "usart.h"
#include "Timer.h"
#include "Motor_Td.h"

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
    uint32_t status = USART2->SR;
    uint8_t data;
    uint16_t i;

    /* 先读SR再读DR，一次清除RXNE/IDLE/错误，避免IDLE分支再次读DR丢字节。 */
    if (status & (USART_FLAG_RXNE | USART_FLAG_IDLE | USART_FLAG_ORE |
                  USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
    {
        data = (uint8_t)USART2->DR;
        if (status & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
        {
            Motor_Td_RxError();
        }
        else if (status & USART_FLAG_RXNE)
        {
            fifo_enQueue(data);
            Motor_Td_RxByte(data); /* 到位检测专用队列，中断里只存字节。 */
        }
        if (status & USART_FLAG_IDLE)
        {
            /* 保留原rxCmd/rxCount接口，_Td不依赖这个单帧缓存。 */
            rxCount = fifo_queueLength();
            for (i = 0; i < rxCount; i++) rxCmd[i] = fifo_deQueue();
            rxFrameFlag = true;
        }
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

volatile uint8_t  start_flag = 0;
volatile char     start_side = '\0';
volatile uint8_t  start_mode = 0;
#define TJC_KEY_QUEUE_SIZE 16U
static const char tjc_start_prefix[] = "start";
static uint8_t tjc_idx = 0;
static char tjc_side;
static uint32_t tjc_last_tick;
static volatile uint8_t key_queue[TJC_KEY_QUEUE_SIZE];
static volatile uint8_t key_write = 0;
static volatile uint8_t key_read = 0;
volatile uint32_t tjc_key_dropped = 0;

/* 从按键队列取出一条完整事件。side 输出 L/R，mode 输出 1/2。
 * 返回 1 表示取到事件；返回 0 表示队列为空或指针无效。
 * USART6 中断负责入队，主循环负责出队，有效容量为 15 条。 */
uint8_t TJC_GetStartKey(char *side, uint8_t *mode)
{
    uint8_t key;
    if (side == 0 || mode == 0 || key_read == key_write) return 0;
    key = key_queue[key_read];
    *side = (key & 0x80U) ? 'R' : 'L';
    *mode = key & 0x7FU;
    key_read = (uint8_t)((key_read + 1U) % TJC_KEY_QUEUE_SIZE);
    return 1;
}

/* 串口屏：USART6，PC6=TX，PC7=RX，9600 8N1。沿用 main 的中断分组。 */
void TJC_USART6_Init(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef n;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &g);
    USART_StructInit(&u);
    u.USART_BaudRate = 9600;
    u.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART6, &u);
    key_read = key_write = tjc_idx = 0;
    start_flag = start_mode = 0;
    start_side = '\0';
    tjc_key_dropped = 0;
    tjc_last_tick = GetTick();
    n.NVIC_IRQChannel = USART6_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 1;
    n.NVIC_IRQChannelSubPriority = 2;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
    (void)USART6->SR;
    (void)USART6->DR;
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART6, ENABLE);
}

/* 按键 startL1/startR1/startL2/startR2，字节间超时 200ms。 */
static void TJC_ReceiveByte(char ch)
{
    uint32_t now = GetTick();
    uint8_t next;
    if ((uint32_t)(now - tjc_last_tick) > 200U) tjc_idx = 0;
    tjc_last_tick = now;

    if (tjc_idx < sizeof(tjc_start_prefix) - 1U)
    {
        if (ch == tjc_start_prefix[tjc_idx]) tjc_idx++;
        else tjc_idx = (ch == 's') ? 1U : 0U;
    }
    else if (tjc_idx == sizeof(tjc_start_prefix) - 1U)
    {
        if (ch == 'L' || ch == 'R')
        {
            tjc_side = ch;
            tjc_idx++;
        }
        else tjc_idx = (ch == 's') ? 1U : 0U;
    }
    else
    {
        if (ch == '1' || ch == '2')
        {
            start_side = tjc_side;
            start_mode = (uint8_t)(ch - '0');
            start_flag = 1;
            next = (uint8_t)((key_write + 1U) % TJC_KEY_QUEUE_SIZE);
            if (next != key_read)
            {
                key_queue[key_write] = (uint8_t)(start_mode |
                                        ((tjc_side == 'R') ? 0x80U : 0U));
                key_write = next;
            }
            else tjc_key_dropped++;
        }
        tjc_idx = (ch == 's') ? 1U : 0U;
    }
}

/* 屏幕串口接收中断，由硬件自动调用；读取字符并交给按键解析函数。 */
void USART6_IRQHandler(void)
{
    /* 先读状态寄存器再读数据寄存器，可清除接收溢出标志。 */
    uint32_t status = USART6->SR;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE))
    {
        char ch = (char)USART_ReceiveData(USART6);
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) tjc_idx = 0;
        else if (status & USART_SR_RXNE) TJC_ReceiveByte(ch);
    }
}

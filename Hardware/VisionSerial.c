#include "stm32f4xx.h"
#include "VisionSerial.h"

#define RX_BUF_SIZE 512

static volatile uint8_t rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;  /* 中断写入位置 */
static volatile uint16_t rxTail = 0;  /* 主循环读取位置，满缓冲时丢新字节 */
static volatile uint8_t rxLost = 0;   /* 丢字节或硬件接收错误，整段数据作废 */
static char frame[VISION_PACKET_SIZE];
static uint16_t frameLength = 0;
static uint8_t receiving = 0;

/* 丢弃尚未处理的字节和半帧，防止模式切换后误用上一阶段的数据。
 * 这里只短暂关闭UART4中断，不影响USART1。
 */
void VisionSerial_ResetReceiver(void)
{
    NVIC_DisableIRQ(UART4_IRQn);
    rxTail = rxHead;
    rxLost = 0;
    receiving = 0;
    frameLength = 0;
    NVIC_EnableIRQ(UART4_IRQn);
}

void VisionSerial_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* UART4在APB1总线，PA0/PA1使用AF8复用功能。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(UART4, &USART_InitStructure);

    /* main使用Group_2中断分组。 */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    UART4->SR; UART4->DR;
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

/* 中断只收字节，不解析字符串、不打印、不控制底盘。
 * 先读SR再读DR，也能清除溢出等接收错误；出现错误则丢弃相关数据。
 */
void UART4_IRQHandler(void)
{
    uint32_t status = UART4->SR;
    uint8_t d;
    uint16_t next;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        d = (uint8_t)UART4->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
        {
            rxLost = 1;
            return;
        }
        next = (rxHead + 1) % RX_BUF_SIZE;
        if (next != rxTail)
        {
            rxBuf[rxHead] = d;
            rxHead = next;
        }
        else rxLost = 1;
    }
}

static int16_t VisionSerial_Read(void)
{
    int16_t c;
    if (rxTail == rxHead) return -1;
    c = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return c;
}

static void VisionSerial_SendByte(uint8_t byte)
{
    USART_SendData(UART4, byte);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
}

/* 按长度发送完整帧。
 * 例如data指向两个字符m1且length为2时，发出@m1FFF。
 * 帧头和帧尾由通信层添加，应用层只提供中间内容。
 */
void VisionSerial_SendPacket(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    VisionSerial_SendByte('@');
    for (i = 0; i < length; i++) VisionSerial_SendByte(data[i]);
    VisionSerial_SendByte('F');
    VisionSerial_SendByte('F');
    VisionSerial_SendByte('F');
}

int8_t VisionSerial_ReceivePacket(char *packet)
{
    int16_t c;
    uint16_t i;
    /* 丢字节属于通信错误，只上报给应用层，由应用层决定数据如何失效。 */
    if (rxLost)
    {
        VisionSerial_ResetReceiver();
        return -1;
    }
    while ((c = VisionSerial_Read()) != -1)
    {
        if (rxLost)
        {
            VisionSerial_ResetReceiver();
            return -1;
        }
        if (c == '@')
        {
            receiving = 1;
            frameLength = 0;
        }
        else if (receiving)
        {
            if (frameLength >= VISION_PACKET_SIZE - 1)
            {
                receiving = 0;
                frameLength = 0;
                continue;
            }
            frame[frameLength++] = (char)c;
            if (frameLength >= 3 && frame[frameLength - 3] == 'F' &&
                frame[frameLength - 2] == 'F' && frame[frameLength - 1] == 'F')
            {
                frameLength -= 3;
                for (i = 0; i < frameLength; i++) packet[i] = frame[i];
                packet[frameLength] = '\0';
                receiving = 0;
                frameLength = 0;
                return 1;
            }
        }
    }
    return 0;
}


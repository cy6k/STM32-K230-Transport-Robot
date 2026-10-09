#include "stm32f4xx.h"
#include "Serial.h"
#include "Timer.h"

/**
  * 串口1（PA9=TX, PA10=RX, 115200-8-N-1，DAP调试口）
  *
  * 用法仿 Arduino Serial：
  *   Serial_Available()          —— 有没有数据可读
  *   Serial_Read()               —— 读一个字符（无数据返回-1）
  *   Serial_ParseInt()           —— 读一个整数（可带负号，自动跳过非数字）
  *   Serial_PrintStr/Int/Float   —— 打印字符串/整数/浮点(指定小数位)
  *   Serial_Println()            —— 换行
  *
  * 接收：RXNE中断入环形缓冲；ParseInt靠"100ms没有新字节"判定数字结束
  * 计时：Serial_ParseInt的超时依赖GetTick()，所以main必须先调Timer_Init()
  */

#define RX_BUF_SIZE   128   /* 环形缓冲大小 */

static volatile uint8_t  rxBuf[RX_BUF_SIZE];
static volatile uint16_t rxHead = 0;            /* 中断写位置 */
static volatile uint16_t rxTail = 0;            /* 主循环读位置 */

/**
  * 函    数：串口1初始化（PA9=TX, PA10=RX, 115200-8-N-1）
  * 参    数：无
  * 返 回 值：无
  * 备    注：NVIC分组由main的NVIC_PriorityGroupConfig决定（Group_2）
  */
void Serial_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* USART1: APB2; GPIOA: AHB1 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /*GPIO初始化（F4：复用+推挽/上拉+绑定AF7，漏了PinAFConfig发不出数据）*/
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    /*USART1配置：115200-8-N-1*/
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    /*NVIC：抢占1/子1，低于电机侧USART2（子0）*/
    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*清残留标志，使能中断和外设*/
    USART1->SR; USART1->DR;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：串口1中断（接收入环形缓冲）
  * 备    注：实体在本模块，it.c不转发；满了丢新字节，不覆盖旧数据
  */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t d = (uint8_t)USART_ReceiveData(USART1);   /*读DR自动清RXNE*/
        uint16_t next = (rxHead + 1) % RX_BUF_SIZE;
        if (next != rxTail)
        {
            rxBuf[rxHead] = d;
            rxHead = next;
        }
    }
}

/**
  * 函    数：检查是否有数据可读（仿Serial.available）
  * 返 回 值：可读的字节数
  */
uint16_t Serial_Available(void)
{
    return (uint16_t)((rxHead - rxTail + RX_BUF_SIZE) % RX_BUF_SIZE);
}

/**
  * 函    数：读取一个字节（仿Serial.read）
  * 返 回 值：有数据返回该字节；无数据返回-1
  */
int16_t Serial_Read(void)
{
    int16_t c;
    if (rxTail == rxHead) return -1;
    c = rxBuf[rxTail];
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return c;
}

/**
  * 函    数：读取一个整数（仿Serial.parseInt）
  * 备    注：跳过非数字字符；遇'-'或数字开始累积；100ms没有新字节视为结束
  *           支持负数。串口助手勾不勾"发送新行"都能用
  */
long Serial_ParseInt(void)
{
    long num = 0;
    uint8_t neg = 0, started = 0;
    uint32_t t0 = GetTick();

    while (GetTick() - t0 < 100)                    /*100ms无新数据=数字读完*/
    {
        if (rxTail != rxHead)
        {
            char c = (char)Serial_Read();
            t0 = GetTick();

            if (c == '-' && !started)      { neg = 1; started = 1; }
            else if (c >= '0' && c <= '9') { num = num * 10 + (c - '0'); started = 1; }
            else if (started)              break;      /*数字后面跟了别的字符=结束*/
            /*还没开始累积就遇到其他字符：跳过*/
        }
    }
    return neg ? -num : num;
}

/**
  * 函    数：发送一个字节（底层）
  */
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/**
  * 函    数：打印字符串（仿Serial.print("...")）
  * 备    注：源文件是GBK编码，串口助手需按ANSI/GBK解码，否则中文乱码
  */
void Serial_PrintStr(const char *str)
{
    while (*str != '\0')
    {
        Serial_SendByte((uint8_t)*str++);
    }
}

/**
  * 函    数：打印整数（仿Serial.print(123)）
  */
void Serial_PrintInt(long num)
{
    char buf[12];                                   /*-2147483648最多11字符+结尾*/
    uint8_t i = 0, a = 0, b;
    char tmp;

    if (num < 0) { Serial_SendByte('-'); num = -num; }
    if (num == 0) buf[i++] = '0';
    while (num > 0) { buf[i++] = (char)('0' + num % 10); num /= 10; }

    for (a = 0, b = i; a < --b; a++)                /*首尾交换，倒序转正序*/
    {
        tmp = buf[a]; buf[a] = buf[b]; buf[b] = tmp; 
    }
    buf[i] = '\0';
    Serial_PrintStr(buf);
}

/**
  * 函    数：打印浮点数（仿Serial.print(x, 3)）
  * 参    数：f 要打印的数；digits 小数位数(0~6)
  * 备    注：手动拆整数/小数部分打印——MicroLIB的printf对%f不可靠，别用
  */
void Serial_PrintFloat(float f, uint8_t digits)
{
    float round = 0.5f;
    uint8_t i;
    long ipart;
    float frac;

    if (f < 0) { Serial_SendByte('-'); f = -f; }

    for (i = 0; i < digits; i++) round *= 0.1f;     /*四舍五入预留量*/
    f += round;

    ipart = (long)f;
    frac = f - (float)ipart;
    if (frac >= 1.0f) { ipart++; frac -= 1.0f; }    /*进位修正*/

    Serial_PrintInt(ipart);

    if (digits > 0)
    {
        Serial_SendByte('.');
        for (i = 0; i < digits; i++)
        {
            frac *= 10.0f;
            Serial_SendByte((uint8_t)('0' + (long)frac));
            frac -= (long)frac;
        }
    }
}

/**
  * 函    数：换行（仿Serial.println，输出\r\n）
  */
void Serial_Println(void)
{
    Serial_PrintStr("\r\n");
}

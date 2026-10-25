#include "stm32f4xx.h"
#include "Light.h"

// PA4 推挽输出，高电平亮，低电平灭；默认关闭。
void Light_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 开灯
void Light_on(void)
{
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

// 关灯
void Light_off(void)
{
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

#include "stm32f4xx.h"
#include "Light.h"

#define LIGHT_PWM_STEPS       100U
#define LIGHT_PWM_FREQUENCY   20000U

// PB0 = TIM3_CH3（AF2），高电平亮；TIM3 硬件输出 20kHz PWM。
void Light_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    RCC_ClocksTypeDef RCC_Clocks;
    uint32_t TimerClock;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // 配置 PWM 前保持低电平，默认关闭补光灯。
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TimerClock = RCC_Clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        TimerClock *= 2U;
    }

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler =
        (uint16_t)(TimerClock / (LIGHT_PWM_FREQUENCY * LIGHT_PWM_STEPS) - 1U);
    TIM_TimeBaseStructure.TIM_Period = LIGHT_PWM_STEPS - 1U;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    // CNT < CCR3 时输出高电平；CCR3=0 全关，CCR3=100 全亮。
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    // 在下一个周期更新亮度，避免中途改变当前脉冲。
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_Cmd(TIM3, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 0 关闭，100 常亮，其余值为占空比百分数；在下一周期更新。
void Light_set(uint8_t Brightness)
{
    if (Brightness > LIGHT_PWM_STEPS)
    {
        Brightness = LIGHT_PWM_STEPS;
    }
    TIM_SetCompare3(TIM3, Brightness);
}

// 关闭补光灯。
void Light_off(void)
{
    Light_set(0);
}

#include "stm32f4xx.h"
#include "Light.h"

#define LIGHT_PWM_STEPS       100U
#define LIGHT_PWM_FREQUENCY   1000U
#define LIGHT_TIMER_FREQUENCY 1000000U
#define LIGHT_PERIOD_TICKS    (LIGHT_TIMER_FREQUENCY / LIGHT_PWM_FREQUENCY)

static volatile uint8_t Light_Brightness = 0;
static uint8_t Light_ActiveBrightness = 0;
static uint8_t Light_HighPhase = 0;

// PA4 驱动低端 NMOS：高电平亮，低电平灭；TIM7 中断模拟 1kHz PWM。
void Light_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_ClocksTypeDef RCC_Clocks;
    uint32_t TimerClock;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_DeInit(TIM7);
    Light_Brightness = 0;
    Light_ActiveBrightness = 0;
    Light_HighPhase = 0;
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_GetClocksFreq(&RCC_Clocks);
    TimerClock = RCC_Clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        TimerClock *= 2U;
    }

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler =
        (uint16_t)(TimerClock / LIGHT_TIMER_FREQUENCY - 1U);
    TIM_TimeBaseStructure.TIM_Period = LIGHT_PERIOD_TICKS - 1U;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    // 不使用 ARR 预装载，让下一段高/低电平时长立即生效。
    TIM_ARRPreloadConfig(TIM7, DISABLE);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM7, ENABLE);
}

// 0 关闭，100 常亮，其余值为占空比百分数；在下一周期更新。
void Light_set(uint8_t Brightness)
{
    if (Brightness > LIGHT_PWM_STEPS)
    {
        Brightness = LIGHT_PWM_STEPS;
    }
    Light_Brightness = Brightness;
}

void Light_Control(uint8_t State)
{
    Light_set(State != 0 ? 100 : 0);
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
    {
        TIM_Cmd(TIM7, DISABLE);
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        if (Light_HighPhase)
        {
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
            TIM_SetAutoreload(TIM7,
                (LIGHT_PWM_STEPS - Light_ActiveBrightness) *
                (LIGHT_PERIOD_TICKS / LIGHT_PWM_STEPS) - 1U);
            Light_HighPhase = 0;
        }
        else
        {
            // 一个周期内保持同一亮度，避免修改参数时高低时长不匹配。
            Light_ActiveBrightness = Light_Brightness;
            if (Light_ActiveBrightness == 0)
            {
                GPIO_ResetBits(GPIOA, GPIO_Pin_4);
                TIM_SetAutoreload(TIM7, LIGHT_PERIOD_TICKS - 1U);
            }
            else if (Light_ActiveBrightness == LIGHT_PWM_STEPS)
            {
                GPIO_SetBits(GPIOA, GPIO_Pin_4);
                TIM_SetAutoreload(TIM7, LIGHT_PERIOD_TICKS - 1U);
            }
            else
            {
                GPIO_SetBits(GPIOA, GPIO_Pin_4);
                TIM_SetAutoreload(TIM7,
                    Light_ActiveBrightness *
                    (LIGHT_PERIOD_TICKS / LIGHT_PWM_STEPS) - 1U);
                Light_HighPhase = 1;
            }
        }
        TIM_SetCounter(TIM7, 0);
        TIM_Cmd(TIM7, ENABLE);
    }
}

#include "stm32f4xx.h"
#include "Servo.h"

/*
 * PA5 -> TIM2_CH1
 * 舵机控制：50Hz PWM，周期 20ms
 * TIM2 时钟一般 84MHz，预分频 84-1 后计数频率 1MHz
 * 所以 CCR 值直接等于脉宽微秒数：
 * 500  -> 0.5ms
 * 1500 -> 1.5ms
 * 2500 -> 2.5ms
 */

#define SERVO_TIM               TIM2
#define SERVO_TIM_CLK           RCC_APB1Periph_TIM2

#define SERVO_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define SERVO_GPIO_PORT         GPIOA
#define SERVO_GPIO_PIN          GPIO_Pin_5
#define SERVO_GPIO_PIN_SOURCE   GPIO_PinSource5
#define SERVO_GPIO_AF           GPIO_AF_TIM2

#define SERVO_TIM_PSC           (84 - 1)      // 84MHz -> 1MHz
#define SERVO_TIM_ARR           (20000 - 1)   // 1MHz 计数，20000 -> 20ms

#define SERVO_MIN_PULSE         500           // 0.5ms
#define SERVO_MAX_PULSE         2500          // 2.5ms

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  * 备    注：PA5，TIM2_CH1，50Hz PWM
  */
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 开启 GPIOA 和 TIM2 时钟
    RCC_AHB1PeriphClockCmd(SERVO_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(SERVO_TIM_CLK, ENABLE);

    // PA5 复用为 TIM2_CH1
    GPIO_PinAFConfig(SERVO_GPIO_PORT, SERVO_GPIO_PIN_SOURCE, SERVO_GPIO_AF);

    // GPIO 初始化
    GPIO_InitStructure.GPIO_Pin   = SERVO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStructure);

    // 定时器时基初始化：50Hz
    TIM_TimeBaseStructure.TIM_Prescaler     = SERVO_TIM_PSC;
    TIM_TimeBaseStructure.TIM_Period        = SERVO_TIM_ARR;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);

    // PWM 通道初始化：TIM2_CH1
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = SERVO_MIN_PULSE; // Default: 0 degrees
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(SERVO_TIM, &TIM_OCInitStructure);

    // 使能预装载
    TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(SERVO_TIM, ENABLE);

    // 开启定时器
    TIM_Cmd(SERVO_TIM, ENABLE);
}

/**
  * 函    数：设置舵机脉宽
  * 参    数：pulse_us：脉宽，单位微秒，一般 500~2500
  * 返 回 值：无
  * 备    注：500 对应 0°，1500 对应 90°，2500 对应 180°
  */
void Servo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE)
    {
        pulse_us = SERVO_MIN_PULSE;
    }
    if (pulse_us > SERVO_MAX_PULSE)
    {
        pulse_us = SERVO_MAX_PULSE;
    }

    TIM_SetCompare1(SERVO_TIM, pulse_us);
}

/**
  * 函    数：设置舵机角度
  * 参    数：angle：角度，0~180
  * 返 回 值：无
  * 备    注：按 500us~2500us 对应 0~180° 线性换算
  */
void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulse;

    if (angle > 180)
    {
        angle = 180;
    }

    pulse = SERVO_MIN_PULSE + (uint16_t)((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180);

    TIM_SetCompare1(SERVO_TIM, pulse);
}


/**
  * @brief Set the target angle directly (0-180 degrees).
  */
void Servo_Write(uint8_t angle)
{
    Servo_SetAngle(angle);
}








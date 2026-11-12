#include "stm32f4xx.h"
#include "Key.h"
#include "Delay.h"

//按键初始化：PA4 为按键1，PB1 为按键2
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //开启 GPIOA、GPIOB 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;          	//选择 PA4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;      	//输入模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;      	//下拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);            	//初始化按键1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;          	//切换到 PB1，其余配置相同
    GPIO_Init(GPIOB, &GPIO_InitStructure);            	//初始化按键2

}


//读取按键1：按下返回1，长按不重复
uint8_t Key1_get()
{
	static uint8_t Key_Num =0;  //0：未触发，1：已触发
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 1)
	{
		if(Key_Num==0)  //本次按下尚未处理
		{
			Delay_ms(15);  //按下消抖
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 1)
			{
				Key_Num = 1;  //记录已触发，防止长按重复
				return 1;     //上报一次按下
			}
		}
	}
	else if(Key_Num == 1)  //检测松开
	{
		Delay_ms(15);  //松开消抖
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0)
		{
			Key_Num = 0;  //松开后允许再次触发
		}
	}
	return 0;  //无新的按下动作
}




//读取按键2：按下返回1，长按不重复
uint8_t Key2_get()
{
	static uint8_t Key_Num =0;  //0：未触发，1：已触发
	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1)
	{
		if(Key_Num==0)  //本次按下尚未处理
		{
			Delay_ms(15);  //按下消抖
			if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1)
			{
				Key_Num = 1;  //记录已触发，防止长按重复
				return 1;     //上报一次按下
			}
		}
	}
	else if(Key_Num == 1)  //检测松开
	{
		Delay_ms(15);  //松开消抖
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 0)
		{
			Key_Num = 0;  //松开后允许再次触发
		}
	}
	return 0;  //无新的按下动作
}





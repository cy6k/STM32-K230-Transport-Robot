#include "stm32f4xx.h"
#include <Delay.h>
#include "Timer.h"
#include "usart.h"
#include "Motor.h"
#include "Serial.h"


char command;
long number;
uint32_t Time;
void test(void);


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断分组2
	Timer_Init();						//TIM6 1ms节拍：Serial_ParseInt的超时依赖GetTick()，不能删
	usart_init();						//USART2(PA2=TX / PA3=RX) 115200，接张大头电机
	Serial_Init();						//USART1(PA9=TX / PA10=RX)，DAP调试口
	Delay_ms(500);						//等电机自检完成
	EC_Enable();						//4个电机都使能
	Delay_ms(100);						//等闭环建立，防止第一条位置命令被忽略
	
	Serial_PrintStr("Init OK\r\n");
	
	while (1)
	{
		test();
	}
}




void test(void)
{
	if(Serial_Available())
	{
		command = Serial_Read();
		number = Serial_ParseInt();

		switch(command)
		{
			case 'a':
			Move_QH(number);
			Serial_PrintStr("QH delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'b':
			Move_ZY(number);
			Serial_PrintStr("ZY delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'c':
			Move_Turn(number);
			Serial_PrintStr("TURN delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
		}
	}
}

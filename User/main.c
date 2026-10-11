#include "stm32f4xx.h"
#include <Delay.h>
#include "Timer.h"
#include "usart.h"
#include "Motor.h"
#include "Serial.h"
#include "HWT101.h"


char command;
long number;
uint32_t Time;
void test(void);
void test_run(void);

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断分组2
	Timer_Init();						//TIM6 1ms节拍：Serial_ParseInt的超时依赖GetTick()，不能删
	usart_init();						//USART2(PA2=TX / PA3=RX) 115200，接张大头电机
	Serial_Init();						//USART1(PA9=TX / PA10=RX)，DAP调试口
	HWT101_Init();
	Delay_ms(500);						//等电机自检完成
	EC_Enable();						//4个电机都使能
	Delay_ms(100);						//等闭环建立，防止第一条位置命令被忽略
	
	Serial_PrintStr("Init OK\r\n");
	
	while (1)
	{
		HWT101_ConfigPoll();
		test();
	}
}



void test(void)
{
	if(Serial_Available())
	{
		command = Serial_Read();

		switch(command)
		{
			case 'a':
			number = Serial_ParseInt();
			Move_QH(number);
			Serial_PrintStr("QH delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'b':
			number = Serial_ParseInt();
			Move_ZY(number);
			Serial_PrintStr("ZY delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'c':
			number = Serial_ParseInt();
			Move_Turn(number);
			Serial_PrintStr("TURN delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'd':
			HWT101_ReadAndPrint();
			break;
			
			case 'e':
			test_run();
			break;
			
		}
	}
}



void test_run()
{
	//出
	Move_QH(2000);
	Delay_ms(1500);
	Move_ZY(2000);
	Delay_ms(1500);
	HWT101_ReadAndPrint();
	
	//到二维码
	Move_Turn(4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	//到原料区
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	//到暂存区
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	
	//到粗加工区
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	//再到原料区
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(23000);
	Delay_ms(4000);
	Move_Turn(-4125);
	Delay_ms(1800);
	HWT101_ReadAndPrint();
	
	
	//再到暂存区
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	//再到粗加工区
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(11500);
	Delay_ms(3000);
	HWT101_ReadAndPrint();
	
	//回
	Move_QH(11500);
	Delay_ms(3000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(23000);
	Delay_ms(4000);
	Move_Turn(-4125);
	Delay_ms(1800);
	Move_QH(-2000);
	Delay_ms(1500);
	Move_ZY(2000);
	Delay_ms(1500);
	HWT101_ReadAndPrint();
}































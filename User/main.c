#include "stm32f4xx.h"
#include <Delay.h>
#include "Timer.h"
#include "usart.h"
#include "Motor.h"
#include "Arm.h"
#include "Serial.h"
#include "HWT101.h"
#include "tjc.h"
#include "Servo.h"
#include "Work.h"
#include "VisionSerial.h"
#include "Light.h"

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断分组2
	Timer_Init();						//TIM6 1ms节拍
	usart_init();						//USART2(PA2=TX / PA3=RX) 115200，接张大头电机
	Serial_Init();						//USART1(PA9=TX / PA10=RX)，DAP调试口
	HWT101_Init();						//陀螺仪初始化
	Servo_Init();						//舵机初始化
	Light_Init();						// 补光灯初始化，PA4/TIM7，默认关闭
	VisionSerial_Init();				// UART4：PA0/PA1，115200，连接 K230 视觉模块
	TJC_USART6_Init();					// USART6: PC6/PC7, 9600, 串口屏
	Delay_ms(500);						//等电机自检完成
	EC_Enable();						//底盘4个电机都使能
	Arm_Enable();						//机械臂3个电机使能
	Delay_ms(100);						//等闭环建立，防止第一条位置命令被忽略
	
	Serial_PrintStr("Init OK\r\n");
	Delay_ms(400);						// 加上已有 600ms 延时，等待屏幕上电约 1 秒
	TJC_SetPage(0);
	
	Delay_ms(3000);
	HWT_Read_Angle();					//其他初始化完成后调用，等待角度稳定并记录为零点
	Serial_PrintStr("Waiting for work \r\n");
	Servo_Write(55);					//所有任务初始化完成后，打开爪子，标志可以开始工作
	
	while (1)
	{
		HWT101_ConfigPoll();
		test();

		// 持续轮询屏幕启动消息，点击任意启动按钮即可运行测试
		if (TJC_ReceiveStartL1() == 1)
		{
			test_run1_Td();
		}
		if (TJC_ReceiveStartR1() == 1)
		{
			test_run2_Td();
		}
		if (TJC_ReceiveStartL2() == 1)
		{
//			test_run3();
		}
		if (TJC_ReceiveStartR2() == 1)
		{
//			test_run4();
		}
	}
}













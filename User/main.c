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

char command;
long number;


void test(void);
void test_run1(void);
void test_run2(void);
void TJC_Test(void);


void grap1(void);
void grap2(void);
void grap3(void);
void put1(void);
void put2(void);
void put3(void);


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断分组2
	Timer_Init();						//TIM6 1ms节拍：Serial_ParseInt的超时依赖GetTick()，不能删
	usart_init();						//USART2(PA2=TX / PA3=RX) 115200，接张大头电机
	Serial_Init();						//USART1(PA9=TX / PA10=RX)，DAP调试口
	HWT101_Init();						//陀螺仪初始化
	Servo_Init();						//舵机初始化
	TJC_USART6_Init();					// USART6: PC6/PC7, 9600, 串口屏
	Delay_ms(500);						//等电机自检完成
	EC_Enable();						//底盘4个电机都使能
	Arm_Enable();						//机械臂3个电机使能
	Delay_ms(100);						//等闭环建立，防止第一条位置命令被忽略
	
	Servo_Write(55);					//初始给55度，持平
	Serial_PrintStr("Init OK\r\n");
	Delay_ms(400);						// 加上已有 600ms 延时，等待屏幕上电约 1 秒
	TJC_SetPage(0);
	Serial_PrintStr("Waiting: startL1 / startL2 / startR1 / startR2\r\n");
	
	while (1)
	{
		HWT101_ConfigPoll();
		test();

		// 持续轮询屏幕启动消息，点击任意启动按钮即可运行测试
		if (TJC_ReceiveStartL1() == 1)
		{
			test_run1();
		}
		if (TJC_ReceiveStartR1() == 1)
		{
			test_run2();
		}
		
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
			grap1();
			put1();
			grap2();
			put2();
			grap3();
			put3();
			break;
			
			
/*			case 't':
			Serial_PrintStr("TJC Start\r\n");
			TJC_Test();
			break;
*/
		
			case 'q':
			number = Serial_ParseInt();
			Arm_Turn(number);
			Serial_PrintStr("Arm Turn delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'w':
			number = Serial_ParseInt();
			Arm_SX(number);
			Serial_PrintStr("Arm SX delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'r':
			number = Serial_ParseInt();
			Arm_QH(number);
			Serial_PrintStr("Arm QH delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'f':
			number = Serial_ParseInt();
			Servo_Write(number);
			Serial_PrintStr("Servo delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
		}
	}
}



void test_run1()
{
	//出
	Move_QH(2000);
	Delay_ms(1200);
	Move_ZY(2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
	
	//到二维码
	Move_Turn(4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//到原料区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//到暂存区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	
	//到粗加工区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//再到原料区
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(23000);
	Delay_ms(3500);
	Move_Turn(-4125);
	Delay_ms(1500);
	HWT101_ReadAndPrint();
	
	
	//再到暂存区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//再到粗加工区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//回
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(23000);
	Delay_ms(3500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(-2000);
	Delay_ms(1200);
	Move_ZY(2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
}




void test_run2(void)
{
	//出
	Move_QH(2000);
	Delay_ms(1200);
	Move_ZY(-2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
	
	//到二维码
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(8250);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//到原料区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//到暂存区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	
	//到粗加工区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//再到原料区
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(23000);
	Delay_ms(3500);
	Move_Turn(-4125);
	Delay_ms(1500);
	HWT101_ReadAndPrint();
	
	
	//再到暂存区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//再到粗加工区
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(-4125);
	Delay_ms(1500);
	Move_QH(11500);
	Delay_ms(2500);
	HWT101_ReadAndPrint();
	
	//回
	Move_QH(11500);
	Delay_ms(2500);
	Move_Turn(8250);
	Delay_ms(2500);
	Move_QH(-2000);
	Delay_ms(1200);
	Move_ZY(-2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
}





/* 屏幕启动消息或调试串口发送 t，运行页面和地图测试（约 12 秒）。 */
void TJC_Test(void)
{
	TJC_SetPage(1);                  /* 显示地图页 */
	Delay_ms(2000);

	TJC_UpdateObstacle(2, 3, 1);     /* 坐标(2,3)有障碍物，g32变黑 */
	Delay_ms(2000);

	TJC_UpdateObstacle(2, 3, 0);     /* 障碍物消失，恢复原色 */
	Delay_ms(2000);

	TJC_SetPage(2);                  /* 显示数据页 */
	Delay_ms(2000);

	TJC_Show_TaskCode("135+531+246+642");
	Delay_ms(2000);

	TJC_SetPage(3);                  /* 显示结束页 */
	Delay_ms(2000);

	TJC_ClearObstacles();            /* 清空障碍物记录 */

	// 测试结束后回到 page0，等待下一次屏幕启动消息
	TJC_SetPage(0);
	Serial_PrintStr("Waiting: startL1 / startL2 / startR1 / startR2\r\n");
}


void grap1(void)
{
	Arm_Turn(-700);
	Delay_ms(1000);
	Arm_QH(27300);
	Delay_ms(2000);
	Arm_SX(-10600);
	Delay_ms(1500);
	Servo_Write(1);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);
	Arm_QH(4000);
	Delay_ms(1000);
}



void grap2(void)
{
	Arm_Turn(0);
	Delay_ms(1000);
	Arm_QH(11000);
	Delay_ms(1500);
	Arm_SX(-10600);
	Delay_ms(1500);
	Servo_Write(1);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);
	Arm_QH(4000);
	Delay_ms(1000);
}



void grap3(void)
{
	Arm_Turn(700);
	Delay_ms(1000);
	Arm_QH(27300);
	Delay_ms(2000);
	Arm_SX(-10600);
	Delay_ms(1500);
	Servo_Write(1);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);
	Arm_QH(4000);
	Delay_ms(1000);
}

void put1()
{
	Arm_QH(3800);
	Delay_ms(500);
	Arm_Turn(-1350);
	Delay_ms(1000);
	Arm_SX(-8500);
	Delay_ms(1000);
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);	
}


void put2()
{
	Arm_QH(1800);
	Delay_ms(500);
	Arm_Turn(300);
	Delay_ms(1000);
	Arm_SX(-8500);
	Delay_ms(1000);
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);	
}

void put3()
{
	Arm_QH(3800);
	Delay_ms(500);
	Arm_Turn(1850);
	Delay_ms(1000);
	Arm_SX(-8500);
	Delay_ms(1000);
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX(-3000);
	Delay_ms(1000);	
}









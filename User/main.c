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
#include "Vision_work.h"
#include "Light.h"


char command;
long number;


int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//中断分组2
	Timer_Init();						//TIM6 1ms节拍
	usart_init();						//USART2(PA2=TX / PA3=RX) 115200，接张大头电机
	Serial_Init();						//USART1(PA9=TX / PA10=RX)，DAP调试口
	HWT101_Init();						//陀螺仪初始化
	Servo_Init();						//舵机初始化
	Light_Init();						// 补光灯初始化，PA4，默认关闭
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
	Servo_Write(45);					//所有任务初始化完成后，打开爪子，标志可以开始工作
	
	Delay_ms(3000);						//爪子张开后3秒，再回到范围内
	Arm_Turn(2100);
	
	
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




void test(void)
{
	uint8_t group, i;
	long angle;
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
			
			case 'n':
			angle = Serial_ParseInt();
			number = Serial_ParseInt();
			Move(angle, number);
			Serial_PrintStr("Move angle:");
			Serial_PrintInt(angle);
			Serial_PrintStr(" pulses:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			
			
			case 'i':
			number = Serial_ParseInt();
			Adjust(number);
			Serial_PrintStr("Adjust delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'o':
			number = Serial_ParseInt();
			Light_set(number);
			Serial_PrintStr("Light brightness:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			
			case 'e':
			Serial_PrintStr("Test Start");
/*			number = Serial_ParseInt();
			Vision_SetMode(number);
			Serial_PrintInt(number);
			Serial_Println();
*/
			put1();
			put2();
			put3();
			
			grap1();
			grap2();
			grap3();
			break;
			
			
			
			case 't':
			Serial_PrintStr("Test Start 2\r\n");
//			TJC_Test();
			
			break;

		
			// 视觉调试：j=二维码，k=料盘识别，l=加工模式，p=模式5圆环，m=休息
			case 'j':
			// 在本case内持续接收二维码；发送m退出。
			Vision_SetMode(1);
			Serial_PrintStr("K230 Mode1 QR\r\n");
			while (1)
			{
				if (Serial_Available() && Serial_Read() == 'm')
				{
					Vision_SetMode(4);
					Serial_PrintStr("no work\r\n");
					break;
				}
				HWT101_ConfigPoll();
				if (Vision_Read_QR())
				{
					Vision_SetMode(4);
					Serial_PrintStr("qiehuan");
					Serial_PrintStr("QR: ");
					for (group = 0; group < 4; group++)
					{
						for (i = 0; i < 3; i++)
							Serial_SendByte((uint8_t)('0' + QR_Data[group][i]));
						if (group < 3) Serial_SendByte('+');
					}
					Serial_Println();
					break;
				}
			}
			break;

			case 'k':
			// 在本case内持续接收料盘数据；发送m退出，其余调试命令退出后再发送。
			Vision_SetMode(2);
			Serial_PrintStr("K230 Mode2 Material\r\n");
			while (1)
			{
				if (Serial_Available() && Serial_Read() == 'm')
				{
					Vision_SetMode(4);
					Serial_PrintStr("no work\r\n");
					break;
				}
				HWT101_ConfigPoll();			// 等待期间继续完成陀螺仪配置
				Vision_Read_material();
				Locate(); 						// 需要模式2定位时，再取消本行注释
			}
			break;

			case 'l':
			// 在本case内持续接收圆环数据并定位；发送m退出。
			Vision_SetMode(3);
			Serial_PrintStr("K230 Mode3 Ring\r\n");
			while (1)
			{
				if (Serial_Available() && Serial_Read() == 'm')
				{
					Vision_SetMode(4);
					Serial_PrintStr("no work\r\n");
					break;
				}
				HWT101_ConfigPoll(); 	// 等待期间继续完成陀螺仪配置
				Vision_Read_Ring();
				orient(); 				// 需要只接收打印时，可以注释本行
			}
			break;

			case 'p':
			// 在本case内持续接收模式5圆环数据并定位；发送m退出。
			Vision_SetMode(5);
			Serial_PrintStr("K230 Mode5 Ring\r\n");
			while (1)
			{
				if (Serial_Available() && Serial_Read() == 'm')
				{
					Vision_SetMode(4);
					Serial_PrintStr("no work\r\n");
					break;
				}
				HWT101_ConfigPoll(); 	// 等待期间继续完成陀螺仪配置
				Vision_Read_Ring();
				orient(); 				// 需要只接收打印时，可以注释本行
			}
			break;

			case 'm':
			// 通知 K230 休息。
			Vision_SetMode(4);
			Serial_PrintStr("no work\r\n");
			break;

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
			
			case 'g':
			number = Serial_ParseInt();
			Move_slant1(number);
			Serial_PrintStr("slant1 delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'h':
			number = Serial_ParseInt();
			Move_slant2(number);
			Serial_PrintStr("slant2 delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
		}
	}
}










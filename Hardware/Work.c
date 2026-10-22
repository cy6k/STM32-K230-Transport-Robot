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
#include "Emm_V5.h"
#include "Vision_work.h"
#include "Light.h"


char command;
long number;


void test(void)
{
	uint8_t group, i;
	if(Serial_Available())
	{
		command = Serial_Read();

		switch(command)
		{
			case 'a':
			number = Serial_ParseInt();
//			Move_QH(number);
			Move_QH_Td(number);
			Serial_PrintStr("QH delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'b':
			number = Serial_ParseInt();
//			Move_ZY(number);
			Move_ZY_Td(number);
			Serial_PrintStr("ZY delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;

			case 'c':
			number = Serial_ParseInt();
//			Move_Turn(number);
			Move_Turn_Td(number);
			Serial_PrintStr("TURN delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'd':
			HWT101_ReadAndPrint();
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
//			put_Td();
//			grap_Td();
			catch_11();
			catch_12();
			catch_13();
			catch_21();
			catch_22();
			catch_23();
			catch_31();
			catch_32();
			catch_33();
			break;
			
			
/*			case 't':
			Serial_PrintStr("TJC Start\r\n");
			TJC_Test();
			break;
*/
		
			// 视觉调试：j=二维码，k=料盘识别，l=加工模式，m=休息
			case 'j':
			// 连续两帧二维码一致后保存四组数字，并通知 K230 休息。
			Vision_SetMode(1);
			Vision_Read_QR();
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

			case 'm':
			// 通知 K230 休息。
			Vision_SetMode(4);
			Serial_PrintStr("no work\r\n");
			break;

			case 'q':
			number = Serial_ParseInt();
//			Arm_Turn(number);
			Arm_Turn_Td(number);
			Serial_PrintStr("Arm Turn delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'w':
			number = Serial_ParseInt();
//			Arm_SX(number);
			Arm_SX_Td(number);
			Serial_PrintStr("Arm SX delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'r':
			number = Serial_ParseInt();
//			Arm_QH(number);
			Arm_QH_Td(number);
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
//			Move_slant1(number);
			Move_slant1_Td(number);
			Serial_PrintStr("slant1 delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
			case 'h':
			number = Serial_ParseInt();
//			Move_slant2(number);
			Move_slant2_Td(number);
			Serial_PrintStr("slant2 delta:");
			Serial_PrintInt(number);
			Serial_Println();
			break;
			
		}
	}
}




void test_run1_Td(void)
{
	//出
	Move_slant1_Td(2000);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	
	//到二维码
	Adjust(90);
	Move_QH_Td(11500);
	HWT101_ReadAndPrint();
	
	
	//到原料区
	Move_QH_Td(11500);
	Adjust(0);
	Move_QH_Td(11500);
	HWT101_ReadAndPrint();
	
	
	//到粗加工区
	Adjust(-90);
	Move_QH_Td(23000);
	
	
	//到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	

	//再到原料区
	Move_QH_Td(11500);
	Adjust(180);
	Move_QH_Td(11500);
	
	
	//再到粗加工区
	Adjust(-90);
	Move_QH_Td(23000);
	
	
	//再到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	
	
	//回
	Adjust(180);
	Move_QH_Td(23000);
	Adjust(90);
	Move_QH_Td(11500);
	Adjust(0);
	Move_slant2_Td(2000);
	HWT101_ReadAndPrint();
}





void test_run2_Td(void)
{
	//出
	Move_slant2_Td(-2000);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	
	//到原料区
	Move_QH_Td(11500);
	HWT101_ReadAndPrint();
	
	//到粗加工区
	Adjust(-90);
	Move_QH_Td(23000);
	
	
	//到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	

	//再到原料区
	Move_QH_Td(11500);
	Adjust(180);
	Move_QH_Td(11500);
	
	
	//再到粗加工区
	Adjust(-90);
	Move_QH_Td(23000);
	
	
	//再到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	
	
	//回
	Adjust(180);
	Move_QH_Td(23000);
	Adjust(-90);
	Move_QH_Td(11500);
	Adjust(0);
	Move_slant1_Td(-2000);
	HWT101_ReadAndPrint();
}




void put_Td(void)
{
	//抓取料盘第一个
	Arm_QH_Td(4000);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//放到圆环1号
	Arm_Turn_Td(-1610);
	Arm_QH_Td(19200);
	Arm_SX_Td(-19400);
	Servo_Write(20);
	
	//放完爪子提起来
	Arm_SX_Td(-4000);
	
	
	
	//抓取料盘第二个
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//放到圆环2号
	Arm_Turn_Td(20);
	Arm_QH_Td(14400);
	Arm_SX_Td(-19400);
	Servo_Write(20);
	
	//放完爪子提起来
	Arm_SX_Td(-4000);
	
	
	//抓取料盘第三个
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//放到圆环3号
	Arm_Turn_Td(1680);
	Arm_QH_Td(18300);
	Arm_SX_Td(-19400);
	Servo_Write(20);
	
	//放完爪子提起来
	Arm_SX_Td(-4000);
}



/* 将圆环 1、2、3 的物料依次放回料盘 1、2、3。
 * 调用前车身应已对准圆环，夹爪为空；抓取、放回位置沿用 put_Td()。
 * 与 put_Td() 一样直接调用到位检测函数，按顺序完成抓取和放回。
 */
void grap_Td(void)
{
	//先松开夹爪并抬到空载转移高度
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX_Td(-4000);

	//抓取圆环3号物料
	Arm_Turn_Td(1680);
	Arm_QH_Td(18300);
	Arm_SX_Td(-19400);
	Servo_Write(0);
	Delay_ms(500);

	//抓稳后抬到原 put_Td() 使用的带料转移高度
	Arm_SX_Td(-7000);

	//按原带料路径逆序转回料盘3号，再下降松开放料
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	Servo_Write(20);
	Delay_ms(500);

	//等待松爪完成后抬起，准备下一次动作
	Arm_SX_Td(-4000);


	//抓取圆环2号物料
	Arm_Turn_Td(20);
	Arm_QH_Td(14400);
	Arm_SX_Td(-19400);
	Servo_Write(0);
	Delay_ms(500);

	//抓稳后抬到原 put_Td() 使用的带料转移高度
	Arm_SX_Td(-7000);

	//按原带料路径逆序转回料盘2号，再下降松开放料
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	Servo_Write(20);
	Delay_ms(500);

	//等待松爪完成后抬起，准备下一次动作
	Arm_SX_Td(-4000);


	//抓取圆环1号物料
	Arm_Turn_Td(-1610);
	Arm_QH_Td(19200);
	Arm_SX_Td(-19400);
	Servo_Write(0);
	Delay_ms(500);

	//抓稳后抬到原 put_Td() 使用的带料转移高度
	Arm_SX_Td(-7000);

	//按原带料路径逆序转回料盘1号，再下降松开放料
	Arm_QH_Td(4000);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	Servo_Write(20);
	Delay_ms(500);

	//等待松爪完成后抬起，准备下一次动作
	Arm_SX_Td(-4000);
	
}




//抓取转盘1号位放置料盘1号位
void catch_11(void)
{
	//爪子到转盘1号
	Arm_Turn_Td(-650);
	Arm_QH_Td(28500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘1号
	Arm_QH_Td(3800);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}





//抓取转盘2号位放置料盘2号位
void catch_22(void)
{
	//爪子到转盘2号
	Arm_Turn_Td(-50);
	Arm_QH_Td(12500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘2号
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}




//抓取转盘3号位放置料盘3号位
void catch_33(void)
{
	//爪子到转盘3号
	Arm_Turn_Td(690);
	Arm_QH_Td(28100);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘3号
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}



//抓取转盘1号位放置料盘2号位
void catch_12(void)
{
	//爪子到转盘1号
	Arm_Turn_Td(-650);
	Arm_QH_Td(28500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘2号
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}


//抓取转盘1号位放置料盘3号位
void catch_13(void)
{
	//爪子到转盘1号
	Arm_Turn_Td(-650);
	Arm_QH_Td(28500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘3号
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}


//抓取转盘2号位放置料盘1号位
void catch_21(void)
{
	//爪子到转盘2号
	Arm_Turn_Td(-50);
	Arm_QH_Td(12500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘1号
	Arm_QH_Td(3800);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}


//抓取转盘2号位放置料盘3号位
void catch_23(void)
{
	//爪子到转盘2号
	Arm_Turn_Td(-50);
	Arm_QH_Td(12500);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘3号
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}


//抓取转盘3号位放置料盘1号位
void catch_31(void)
{
	//爪子到转盘3号
	Arm_Turn_Td(690);
	Arm_QH_Td(28100);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘1号
	Arm_QH_Td(3800);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}


//抓取转盘3号位放置料盘2号位
void catch_32(void)
{
	//爪子到转盘3号
	Arm_Turn_Td(690);
	Arm_QH_Td(28100);
	Arm_SX_Td(-11400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX_Td(-3000);
	
	//爪子到料盘2号
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX_Td(-3000);
}

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
#include "VisionSerial.h"
#include "Light.h"



//第一批取料：按2->1->3->2推算位置，每次换位只抓放一个
uint8_t catch_first(void)
{
	uint8_t i, j, source, id, lastId, located = 0, temp, shift;
	uint8_t diskId[3] = {0}; //保留三个原始ID，抓走后仍用于对齐转盘相位
	uint8_t newStop;
	uint32_t start, lastFrame, now;

	for (i = 0; i < 3; i++)
	{
		Vision_SetMode(2);
		lastId = 0;
		start = lastFrame = GetTick();
		while (1)
		{
			HWT101_ConfigPoll();
			if ((uint32_t)(GetTick() - start) >= 23000U)
			{
				Vision_SetMode(4);
				Serial_PrintStr(located ? "Catch wait timeout\r\n" : "Locate timeout\r\n");
				return 0;
			}
			if (!Vision_Read_material()) continue;
			id = (uint8_t)Material_Data[2];
			//只用尚未抓走的物料作为当前转盘位置的参照
			for (j = i; j < 3; j++)
				if (id == QR_Data[0][j]) break;
			if (j == 3) continue;
			now = GetTick();
			newStop = (lastId != 0 && id != lastId) || (uint32_t)(now - lastFrame) >= 600U;
			lastFrame = now;

			if (!located)
			{
				if (Material_Data[0] >= -5 && Material_Data[0] <= 5 &&
					Material_Data[1] >= -5 && Material_Data[1] <= 5)
				{
					Locate();
					located = 1;
					start = lastFrame = GetTick();
				}
				else
				{
					Locate();
					VisionSerial_ResetReceiver();
				}
				lastId = id;
				continue; //定位完成后等下一次新ID，不在当前停留中途抓取
			}

			if (diskId[0] == 0)
			{
				if (lastId == 0 || id == lastId)
				{
					lastId = id;
					continue;
				}
				//中间A变B：1号A、2号B、3号为二维码中剩余的ID
				diskId[0] = lastId;
				diskId[1] = id;
				for (j = 0; j < 3; j++)
					if (QR_Data[0][j] != lastId && QR_Data[0][j] != id) diskId[2] = QR_Data[0][j];
			}
			else
			{
				//抓放期间可能漏看换位，用当前中间ID重新对齐原始位置表
				for (shift = 0; shift < 3 && diskId[1] != id; shift++)
				{
					temp = diskId[0];
					diskId[0] = diskId[1];
					diskId[1] = diskId[2];
					diskId[2] = temp;
				}
			}
			lastId = id;
			// shortcut: 空位不发包，600ms无新帧后再出现视为新停留；若串口会断流需增加转盘状态报文。
			if (!newStop) continue;
			for (source = 0; source < 3; source++)
				if (diskId[source] == QR_Data[0][i]) break;
			break;
		}

		Vision_SetMode(4);
		switch ((source + 1) * 10 + QR_Data[1][i])
		{
			case 11: catch_11(); break;
			case 12: catch_12(); break;
			case 13: catch_13(); break;
			case 21: catch_21(); break;
			case 22: catch_22(); break;
			case 23: catch_23(); break;
			case 31: catch_31(); break;
			case 32: catch_32(); break;
			case 33: catch_33(); break;
			default:
				Serial_PrintStr("Catch position error\r\n");
				return 0;
		}
		//每抓放一个都回准星姿态，重新观察下一次停留
		if (Arm_Turn_Td(0) != MOTOR_TD_OK ||
			Arm_SX_Td(-3000) != MOTOR_TD_OK ||
			Arm_QH_Td(11000) != MOTOR_TD_OK)
		{
			Serial_PrintStr("Catch return error\r\n");
			return 0;
		}
	}
	Serial_PrintStr("Catch first OK\r\n");
	return 1;
}

//第一批放料：先校准圆环，再按二维码第二组顺序放到同号圆环
uint8_t put_first(void)
{
	uint8_t i;
	uint32_t start;

	//摄像头已大致对准圆环，先完成视觉定位再放料
	Vision_SetMode(3);
	start = GetTick();
	while (1)
	{
		HWT101_ConfigPoll();
		if ((uint32_t)(GetTick() - start) >= 23000U)
		{
			Vision_SetMode(4);
			Serial_PrintStr("Put orient timeout\r\n");
			return 0;
		}
		Vision_Read_Ring();
		if (orient()) break;
	}
	Vision_SetMode(4);

	for (i = 0; i < 3; i++)
	{
		switch (QR_Data[1][i])
		{
			case 1: put1(); break;
			case 2: put2(); break;
			case 3: put3(); break;
		}
	}
	return 1;
}


//第一批取回：按二维码第二组顺序，从圆环抓回同号车载料盘
void grap_first(void)
{
	uint8_t i;
	for (i = 0; i < 3; i++)
	{
		switch (QR_Data[1][i])
		{
			case 1: grap1(); break;
			case 2: grap2(); break;
			case 3: grap3(); break;
		}
	}
}




//左出发，无障碍
void test_run1_Td(void)
{
	uint32_t start;

	//出
	Move_slant1_Td(2000);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	//到原料区
	Adjust(90);
	Move_QH_Td(23000);
	Adjust(0);
	Move_QH_Td(11500);
	HWT101_ReadAndPrint();
	
	
	//转动到扫描二维码角度
	Adjust(-90);
	Arm_Turn_Td(9900);
	Arm_SX_Td(-3000);
	Arm_QH_Td(25000);
	
	//扫描二维码,记录数据
	Vision_SetMode(1);
	start = GetTick();
	while (!Vision_Read_QR())
	{
		HWT101_ConfigPoll();
		if ((uint32_t)(GetTick() - start) >= 5000U)
		{
			Vision_SetMode(4);
			Serial_PrintStr("QR timeout\r\n");
			return;
		}
	}
	Vision_SetMode(4);
	
	//为Locate()准备
	Arm_QH_Td(11000);
	Arm_Turn_Td(0);
	Move_QH_Td(-500);
	
	
	//从转盘上抓物料
	catch_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	
	
	//到粗加工区
	Adjust(-90);
	Move_QH_Td(24000);
	
	
	//放物料到粗加工区圆环,放完再取
	Adjust(90);
	Arm_SX_Td(-3000);
	Arm_QH_Td(13000);
	put_first();
	grap_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	
	//到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	
	
	//放物料到暂存区
	Adjust(180);
	Arm_SX_Td(-3000);
	Arm_QH_Td(13000);
	put_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	Adjust(90);


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
	TJC_SetPage(3);
	HWT101_ReadAndPrint();
}


//右出发，无障碍
void test_run2_Td(void)	
{
	uint32_t start;
	
	//出
	Move_slant2_Td(-2000);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	
	//到原料区
	Move_QH_Td(11500);
	HWT101_ReadAndPrint();
	
	//转动到扫描二维码角度
	Adjust(-90);
	Arm_Turn_Td(9900);
	Arm_SX_Td(-3000);
	Arm_QH_Td(25000);
	
	//扫描二维码,记录数据
	Vision_SetMode(1);
	start = GetTick();
	while (!Vision_Read_QR())
	{
		HWT101_ConfigPoll();
		if ((uint32_t)(GetTick() - start) >= 5000U)
		{
			Vision_SetMode(4);
			Serial_PrintStr("QR timeout\r\n");
			return;
		}
	}
	Vision_SetMode(4);
	
	//为Locate()准备
	Arm_QH_Td(11000);
	Arm_Turn_Td(0);
	Move_QH_Td(-1000);
	
	
	//从转盘上抓物料
	catch_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	
	
	//到粗加工区
	Adjust(-90);
	Move_QH_Td(24000);
	
	
	//放物料到粗加工区圆环,放完再取
	Adjust(90);
	Arm_SX_Td(-3000);
	Arm_QH_Td(13000);
	put_first();
	grap_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	
	//到暂存区
	Adjust(0);
	Move_QH_Td(11500);
	Adjust(90);
	Move_QH_Td(11500);
	
	
	//放物料到暂存区
	Adjust(180);
	Arm_SX_Td(-3000);
	Arm_QH_Td(13000);
	put_first();
	Arm_QH_Td(0);
	Arm_Turn_Td(0);
	Adjust(90);


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
	TJC_SetPage(3); 
	HWT101_ReadAndPrint();
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





//抓取料盘1号位放置圆环1号位
void put1(void)
{
	//爪子到料盘1号
	Arm_QH_Td(4000);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到圆环1号
	Arm_Turn_Td(-1610);
	Arm_QH_Td(19200);
	Arm_SX_Td(-19400);
	
	//放
	Servo_Write(20);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}



//抓取料盘2号位放置圆环2号位
void put2(void)
{
	//爪子到料盘2号
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到圆环2号
	Arm_Turn_Td(20);
	Arm_QH_Td(14400);
	Arm_SX_Td(-19400);
	
	//放
	Servo_Write(20);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}




//抓取料盘3号位放置圆环3号位
void put3(void)
{
	//爪子到料盘3号
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到圆环3号
	Arm_Turn_Td(1680);
	Arm_QH_Td(18300);
	Arm_SX_Td(-19400);
	
	//放
	Servo_Write(20);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}






//抓取圆环1号位放置料盘1号位
void grap1(void)
{
	//先松开爪子并提起来
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX_Td(-4000);
	
	//爪子到圆环1号
	Arm_Turn_Td(-1610);
	Arm_QH_Td(19200);
	Arm_SX_Td(-19400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到料盘1号
	Arm_QH_Td(4000);
	Arm_Turn_Td(-1350);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}



//抓取圆环2号位放置料盘2号位
void grap2(void)
{
	//先松开爪子并提起来
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX_Td(-4000);
	
	//爪子到圆环2号
	Arm_Turn_Td(20);
	Arm_QH_Td(14400);
	Arm_SX_Td(-19400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到料盘2号
	Arm_QH_Td(1800);
	Arm_Turn_Td(300);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}


//抓取圆环3号位放置料盘3号位
void grap3(void)
{
	//先松开爪子并提起来
	Servo_Write(20);
	Delay_ms(500);
	Arm_SX_Td(-4000);
	
	//爪子到圆环3号
	Arm_Turn_Td(1680);
	Arm_QH_Td(18300);
	Arm_SX_Td(-19400);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX_Td(-7000);
	
	//爪子到料盘3号
	Arm_QH_Td(3900);
	Arm_Turn_Td(1850);
	Arm_SX_Td(-10300);
	
	//放
	Servo_Write(20);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX_Td(-4000);
}
















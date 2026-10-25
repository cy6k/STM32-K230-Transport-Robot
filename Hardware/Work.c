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



//左出发，无障碍
void test_run1(void)
{
	uint32_t start;

	//出
	Move_slant1(2000);
	Delay_ms(1200);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	//到原料区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(90);
	Move_QH(23000);
	Delay_ms(2800);
	Adjust(0);
	Move_QH(11500);
	Delay_ms(100);
	Arm_Turn(10000);
	Delay_ms(500);
	Arm_SX(-2500);
	Delay_ms(500);
	Arm_QH(25000);
	Delay_ms(1000);
	HWT101_ReadAndPrint();
	
	
	//转动到扫描二维码角度
	Adjust(-90);
	
	
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
	
	
	//为抓料准备
	//在底盘移动过程中，同时调整机械臂状态
	Move_QH(-500);
	Delay_ms(100);
	Arm_QH(11000);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(500);
	
	
	
	//从转盘上抓物料
	catch_first();
	
	
	
	//到粗加工区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(-90);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(300);
	Arm_QH(13000);
	Delay_ms(2000);

	
	
	//放物料到粗加工区圆环,放完再取
	Adjust(90);
	Arm_QH(13000);
	Delay_ms(500);
	put_first();
	grap_first();
	
	

	//到暂存区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(0);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(90);
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(200);
	Arm_QH(13000);
	Delay_ms(1300);
	
	
	
	
	
	//放物料到暂存区
	Adjust(180);
	put_first();
	Adjust(90);


	//再到原料区
	//在底盘移动过程中，同时调整机械臂状态
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(-2500);
	Delay_ms(400);
	Arm_QH(11000);
	Delay_ms(300);
	Arm_Turn(0);
	Delay_ms(1200);
	Adjust(180);
	Move_QH(11500);
	Delay_ms(2000);
	
	
	//准备原料区第二次取料
	Adjust(-90);
	Move_QH(-500);
	Delay_ms(500);
	
	
	//原料区第二次取料
	catch_second();
	
	
	//再到粗加工区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(-90);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(300);
	Arm_QH(13000);
	Delay_ms(2000);

	
	
	//粗加工第二次放料取料
	Adjust(90);
	Arm_QH(13000);
	Delay_ms(500);
	put_second();
	grap_second();
	
	
	//再到暂存区	
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(0);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(90);
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(200);
	Arm_QH(13000);
	Delay_ms(1300);
	
	
	
	//暂存区放第二批物料，重叠
	Adjust(180);
	Arm_QH(13000);
	Delay_ms(500);
	put_third();
	TJC_SetPage(3);
	

	//回
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(180);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_QH(0);
	Delay_ms(300);
	Arm_Turn(0);
	Delay_ms(2000);
	Adjust(90);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(0);
	Move_slant2(2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
}




//右出发，无障碍
void test_run2(void)	
{
	uint32_t start;

	//出
	Move_slant2(-2000);
	Delay_ms(1200);
	Adjust(0);
	HWT101_ReadAndPrint();
	
	//到原料区
	//在底盘移动过程中，同时调整机械臂状态
	Move_QH(11500);
	Delay_ms(100);
	Arm_Turn(10000);
	Delay_ms(500);
	Arm_SX(-2500);
	Delay_ms(500);
	Arm_QH(25000);
	Delay_ms(1000);
	HWT101_ReadAndPrint();
	
	
	//转动到扫描二维码角度
	Adjust(-90);
	
	
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
	
	
	//为抓料准备
	//在底盘移动过程中，同时调整机械臂状态
	Move_QH(-500);
	Delay_ms(100);
	Arm_QH(11000);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(500);
	
	
	
	//从转盘上抓物料
	catch_first();
	
	
	
	//到粗加工区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(-90);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(300);
	Arm_QH(13000);
	Delay_ms(2000);

	
	
	//放物料到粗加工区圆环,放完再取
	Adjust(90);
	Arm_QH(13000);
	Delay_ms(500);
	put_first();
	grap_first();
	
	

	//到暂存区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(0);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(90);
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(200);
	Arm_QH(13000);
	Delay_ms(1300);
	
	
	
	
	
	//放物料到暂存区
	Adjust(180);
	put_first();
	Adjust(90);


	//再到原料区
	//在底盘移动过程中，同时调整机械臂状态
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(-2500);
	Delay_ms(400);
	Arm_QH(11000);
	Delay_ms(300);
	Arm_Turn(0);
	Delay_ms(1200);
	Adjust(180);
	Move_QH(11500);
	Delay_ms(2000);
	
	
	//准备原料区第二次取料
	Adjust(-90);
	Move_QH(-500);
	Delay_ms(500);
	
	
	//原料区第二次取料
	catch_second();
	
	
	//再到粗加工区
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(-90);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(300);
	Arm_QH(13000);
	Delay_ms(2000);

	
	
	//粗加工第二次放料取料
	Adjust(90);
	Arm_QH(13000);
	Delay_ms(500);
	put_second();
	grap_second();
	
	
	//再到暂存区	
	Adjust(0);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(90);
	Move_QH(11500);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_Turn(0);
	Delay_ms(200);
	Arm_QH(13000);
	Delay_ms(1300);
	
	
	
	//暂存区放第二批物料，重叠
	Adjust(180);
	Arm_QH(13000);
	Delay_ms(500);
	put_third();
	TJC_SetPage(3);
	

	//回
	//在底盘移动过程中，同时调整机械臂状态
	Adjust(180);
	Move_QH(23000);
	Delay_ms(100);
	Arm_SX(0);
	Delay_ms(400);
	Arm_QH(0);
	Delay_ms(300);
	Arm_Turn(0);
	Delay_ms(2000);
	Adjust(-90);
	Move_QH(11500);
	Delay_ms(2000);
	Adjust(0);
	Move_slant1(-2000);
	Delay_ms(1200);
	HWT101_ReadAndPrint();
}




//第一批取料：定位后首次新ID建立位置关系抓第一个，下次新ID出现后连续抓剩余两个
// shortcut: 第三抓按实测预判转盘再换位一次，转速或停留时间改变时需重新确认。
uint8_t catch_first(void)
{
	uint8_t i, j, source, id, lastId = 0, located = 0, temp, shift;
	uint8_t diskId[3] = {0}; //保留三个原始ID，抓走后仍用于对齐转盘相位
	uint32_t start;

	for (i = 0; i < 3; i++)
	{
		if (i < 2)
		{
			Vision_SetMode(2);
			start = GetTick();
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
				for (j = i; j < 3; j++)
					if (id == QR_Data[0][j]) break;
				if (j == 3) continue;

				if (!located)
				{
					if (Material_Data[0] >= -5 && Material_Data[0] <= 5 &&
						Material_Data[1] >= -5 && Material_Data[1] <= 5)
					{
						Locate();
						located = 1;
						start = GetTick();
					}
					else
					{
						Locate();
						VisionSerial_ResetReceiver();
					}
					lastId = id;
					continue;
				}

				if (id == lastId) continue;
				if (diskId[0] == 0)
				{
					//首次换位：上次ID到1号位，本次ID在2号位，剩余ID在3号位
					diskId[0] = lastId;
					diskId[1] = id;
					for (j = 0; j < 3; j++)
						if (QR_Data[0][j] != lastId && QR_Data[0][j] != id) diskId[2] = QR_Data[0][j];
				}
				else
				{
					//中间空位不发包，按下次可见ID对齐，兼容跳过一次空位停留
					for (shift = 0; shift < 3 && diskId[1] != id; shift++)
					{
						temp = diskId[0];
						diskId[0] = diskId[1];
						diskId[1] = diskId[2];
						diskId[2] = temp;
					}
				}
				lastId = id;
				break;
			}
			Vision_SetMode(4);
		}

		for (source = 0; source < 3; source++)
			if (diskId[source] == QR_Data[0][i]) break;
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

		//第一个放好后恢复识别姿态，第三个放好后保持当前位置。
		if (i == 0)
		{
			Arm_Turn(0);
			Delay_ms(200);
			Arm_SX(-2500);
			Delay_ms(200);
			Arm_QH(11000);
			Delay_ms(200);
		}

		if (i == 1)
		{
			Delay_ms(2000);
			// 第二个放好时转盘已再换位一次，更新位置后直接抓第三个。
			temp = diskId[0];
			diskId[0] = diskId[1];
			diskId[1] = diskId[2];
			diskId[2] = temp;
			continue;
		}

	}
	Serial_PrintStr("Catch first OK\r\n");
	return 1;
}



//第二批取料：定位后首次新ID建立位置关系抓第一个，下次新ID出现后连续抓剩余两个
// shortcut: 第三抓按实测预判转盘再换位一次，转速或停留时间改变时需重新确认。
uint8_t catch_second(void)
{
	uint8_t i, j, source, id, lastId = 0, located = 0, temp, shift;
	uint8_t diskId[3] = {0}; //保留三个原始ID，抓走后仍用于对齐转盘相位
	uint32_t start;

	for (i = 0; i < 3; i++)
	{
		if (i < 2)
		{
			Vision_SetMode(2);
			start = GetTick();
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
				for (j = i; j < 3; j++)
					if (id == QR_Data[2][j]) break;
				if (j == 3) continue;

				if (!located)
				{
					if (Material_Data[0] >= -5 && Material_Data[0] <= 5 &&
						Material_Data[1] >= -5 && Material_Data[1] <= 5)
					{
						Locate();
						located = 1;
						start = GetTick();
					}
					else
					{
						Locate();
						VisionSerial_ResetReceiver();
					}
					lastId = id;
					continue;
				}

				if (id == lastId) continue;
				if (diskId[0] == 0)
				{
					//首次换位：上次ID到1号位，本次ID在2号位，剩余ID在3号位
					diskId[0] = lastId;
					diskId[1] = id;
					for (j = 0; j < 3; j++)
						if (QR_Data[2][j] != lastId && QR_Data[2][j] != id) diskId[2] = QR_Data[2][j];
				}
				else
				{
					//中间空位不发包，按下次可见ID对齐，兼容跳过一次空位停留
					for (shift = 0; shift < 3 && diskId[1] != id; shift++)
					{
						temp = diskId[0];
						diskId[0] = diskId[1];
						diskId[1] = diskId[2];
						diskId[2] = temp;
					}
				}
				lastId = id;
				break;
			}
			Vision_SetMode(4);
		}

		for (source = 0; source < 3; source++)
			if (diskId[source] == QR_Data[2][i]) break;
		switch ((source + 1) * 10 + QR_Data[3][i])
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

		//第一个放好后恢复识别姿态，第三个放好后保持当前位置。
		if (i == 0)
		{
			Arm_Turn(0);
			Delay_ms(200);
			Arm_SX(-2500);
			Delay_ms(200);
			Arm_QH(11000);
			Delay_ms(200);
		}

		if (i == 1)
		{
			Delay_ms(2000);
			// 第二个放好时转盘已再换位一次，更新位置后直接抓第三个。
			temp = diskId[0];
			diskId[0] = diskId[1];
			diskId[1] = diskId[2];
			diskId[2] = temp;
			continue;
		}
	}
	Serial_PrintStr("Catch second OK\r\n");
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




//第二批粗加工放料：先校准圆环，再按二维码第四组顺序放到同号圆环
uint8_t put_second(void)
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
		switch (QR_Data[3][i])
		{
			case 1: put1(); break;
			case 2: put2(); break;
			case 3: put3(); break;
		}
	}
	return 1;
}



//第二批粗加工取回：按二维码第四组顺序，从圆环抓回同号车载料盘
void grap_second(void)
{
	uint8_t i;
	for (i = 0; i < 3; i++)
	{
		switch (QR_Data[3][i])
		{
			case 1: grap1(); break;
			case 2: grap2(); break;
			case 3: grap3(); break;
		}
	}
}




//第二批暂存区叠放：按第二批颜色顺序，从车载料盘取料，叠到第一批同色物料上。
uint8_t put_third(void)
{
	uint8_t i, j, target[3];
	uint32_t start;

	//两批颜色、位置均不能重复，先检查完整任务，避免抓起后才发现无法叠放。
	for (i = 0; i < 3; i++)
	{
		if (QR_Data[0][i] < 1 || QR_Data[0][i] > 6 ||
			QR_Data[1][i] < 1 || QR_Data[1][i] > 3 ||
			QR_Data[2][i] < 1 || QR_Data[2][i] > 6 ||
			QR_Data[3][i] < 1 || QR_Data[3][i] > 3)
		{
			Serial_PrintStr("Stack task error\r\n");
			return 0;
		}
		for (j = 0; j < i; j++)
		{
			if (QR_Data[0][i] == QR_Data[0][j] || QR_Data[1][i] == QR_Data[1][j] ||
				QR_Data[2][i] == QR_Data[2][j] || QR_Data[3][i] == QR_Data[3][j])
			{
				Serial_PrintStr("Stack task error\r\n");
				return 0;
			}
		}
	}

	//第四组是车载取料位；按第三组颜色，在第一组中查找第二组的同色圆环位。
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
			if (QR_Data[2][i] == QR_Data[0][j]) break;
		if (j == 3)
		{
			Serial_PrintStr("Stack color error\r\n");
			return 0;
		}
		target[i] = QR_Data[1][j];
	}

	Vision_SetMode(3);
	start = GetTick();
	while (1)
	{
		HWT101_ConfigPoll();
		if ((uint32_t)(GetTick() - start) >= 23000U)
		{
			Vision_SetMode(4);
			Serial_PrintStr("Stack orient timeout\r\n");
			return 0;
		}
		Vision_Read_Ring();
		if (orient()) break;
	}
	Vision_SetMode(4);

	for (i = 0; i < 3; i++)
	{
		switch (QR_Data[3][i] * 10 + target[i])
		{
			//车载料盘1号取料，叠放到圆环1号的同色物料上。
			case 11:
				Arm_QH(4000);
				Delay_ms(400);
				Arm_Turn(-1350);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(19200);
				Delay_ms(400);
				Arm_Turn(-1610);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘1号取料，叠放到圆环2号的同色物料上。
			case 12:
				Arm_QH(4000);
				Delay_ms(400);
				Arm_Turn(-1350);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(14400);
				Delay_ms(400);
				Arm_Turn(20);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘1号取料，叠放到圆环3号的同色物料上。
			case 13:
				Arm_QH(4000);
				Delay_ms(400);
				Arm_Turn(-1350);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(18300);
				Delay_ms(400);
				Arm_Turn(1680);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘2号取料，叠放到圆环1号的同色物料上。
			case 21:
				Arm_QH(1800);
				Delay_ms(400);
				Arm_Turn(300);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(19200);
				Delay_ms(400);
				Arm_Turn(-1610);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘2号取料，叠放到圆环2号的同色物料上。
			case 22:
				Arm_QH(1800);
				Delay_ms(400);
				Arm_Turn(300);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(14400);
				Delay_ms(400);
				Arm_Turn(20);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘2号取料，叠放到圆环3号的同色物料上。
			case 23:
				Arm_QH(1800);
				Delay_ms(400);
				Arm_Turn(300);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(18300);
				Delay_ms(400);
				Arm_Turn(1680);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘3号取料，叠放到圆环1号的同色物料上。
			case 31:
				Arm_QH(3900);
				Delay_ms(400);
				Arm_Turn(1850);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(19200);
				Delay_ms(400);
				Arm_Turn(-1610);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘3号取料，叠放到圆环2号的同色物料上。
			case 32:
				Arm_QH(3900);
				Delay_ms(400);
				Arm_Turn(1850);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(14400);
				Delay_ms(400);
				Arm_Turn(20);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

			//车载料盘3号取料，叠放到圆环3号的同色物料上。
			case 33:
				Arm_QH(3900);
				Delay_ms(400);
				Arm_Turn(1850);
				Delay_ms(300);
				Arm_SX(-10300);
				Delay_ms(500);
				Servo_Write(0);
				Delay_ms(500);
				Arm_SX(-7000);
				Delay_ms(300);
				Arm_QH(18300);
				Delay_ms(400);
				Arm_Turn(1680);
				Delay_ms(200);
				Arm_SX(-13600);
				Delay_ms(500);
				Servo_Write(25);
				Delay_ms(500);
				Arm_SX(-4000);
				Delay_ms(500);
				break;

		}
	}
	return 1;
}




//抓取转盘1号位放置料盘1号位
void catch_11(void)
{
	//爪子到转盘1号
	Arm_Turn(-650);
	Delay_ms(300);
	Arm_QH(28500);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘1号
	Arm_QH(3800);
	Delay_ms(450);
	Arm_Turn(-1350);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘1号位放置料盘2号位
void catch_12(void)
{
	//爪子到转盘1号
	Arm_Turn(-650);
	Delay_ms(300);
	Arm_QH(28500);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘2号
	Arm_QH(1800);
	Delay_ms(450);
	Arm_Turn(300);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘1号位放置料盘3号位
void catch_13(void)
{
	//爪子到转盘1号
	Arm_Turn(-650);
	Delay_ms(300);
	Arm_QH(28500);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘3号
	Arm_QH(3900);
	Delay_ms(450);
	Arm_Turn(1850);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘2号位放置料盘1号位
void catch_21(void)
{
	//爪子到转盘2号
	Arm_Turn(-50);
	Delay_ms(300);
	Arm_QH(12500);
	Delay_ms(300);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘1号
	Arm_QH(3800);
	Delay_ms(300);
	Arm_Turn(-1350);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘2号位放置料盘2号位
void catch_22(void)
{
	//爪子到转盘2号
	Arm_Turn(-50);
	Delay_ms(300);
	Arm_QH(12500);
	Delay_ms(300);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘2号
	Arm_QH(1800);
	Delay_ms(300);
	Arm_Turn(300);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘2号位放置料盘3号位
void catch_23(void)
{
	//爪子到转盘2号
	Arm_Turn(-50);
	Delay_ms(300);
	Arm_QH(12500);
	Delay_ms(300);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘3号
	Arm_QH(3900);
	Delay_ms(300);
	Arm_Turn(1850);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘3号位放置料盘1号位
void catch_31(void)
{
	//爪子到转盘3号
	Arm_Turn(690);
	Delay_ms(300);
	Arm_QH(28100);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘1号
	Arm_QH(3800);
	Delay_ms(450);
	Arm_Turn(-1350);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘3号位放置料盘2号位
void catch_32(void)
{
	//爪子到转盘3号
	Arm_Turn(690);
	Delay_ms(300);
	Arm_QH(28100);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘2号
	Arm_QH(1800);
	Delay_ms(450);
	Arm_Turn(300);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取转盘3号位放置料盘3号位
void catch_33(void)
{
	//爪子到转盘3号
	Arm_Turn(690);
	Delay_ms(300);
	Arm_QH(28100);
	Delay_ms(450);
	Arm_SX(-11400);
	Delay_ms(450);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
	
	//爪子到料盘3号
	Arm_QH(3900);
	Delay_ms(450);
	Arm_Turn(1850);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(350);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
		
	//爪子提高到一定高度
	Arm_SX(-2500);
	Delay_ms(250);
}


//抓取料盘1号位放置圆环1号位
void put1(void)
{
	//爪子到料盘1号
	Arm_QH(4000);
	Delay_ms(400);
	Arm_Turn(-1350);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(300);
	
	//爪子到圆环1号
	Arm_Turn(-1610);
	Delay_ms(200);
	Arm_QH(19200);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(500);
}


//抓取料盘2号位放置圆环2号位
void put2(void)
{
	//爪子到料盘2号
	Arm_QH(1800);
	Delay_ms(400);
	Arm_Turn(300);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(300);
	
	//爪子到圆环2号
	Arm_Turn(20);
	Delay_ms(200);
	Arm_QH(14400);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(500);
}


//抓取料盘3号位放置圆环3号位
void put3(void)
{
	//爪子到料盘3号
	Arm_QH(3900);
	Delay_ms(400);
	Arm_Turn(1850);
	Delay_ms(300);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(300);
	
	//爪子到圆环3号
	Arm_Turn(1680);
	Delay_ms(200);
	Arm_QH(18300);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(500);
}


//抓取圆环1号位放置料盘1号位
void grap1(void)
{
	//先松开爪子并提起来
	Servo_Write(25);
	Arm_SX(-4000);
	Delay_ms(100);
	
	//爪子到圆环1号
	Arm_Turn(-1610);
	Delay_ms(300);
	Arm_QH(19200);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(450);
	
	//爪子到料盘1号
	Arm_QH(4000);
	Delay_ms(400);
	Arm_Turn(-1350);
	Delay_ms(200);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(350);
}


//抓取圆环2号位放置料盘2号位
void grap2(void)
{
	//先松开爪子并提起来
	Servo_Write(25);
	Arm_SX(-4000);
	Delay_ms(100);
	
	//爪子到圆环2号
	Arm_Turn(20);
	Delay_ms(300);
	Arm_QH(14400);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(450);
	
	//爪子到料盘2号
	Arm_QH(1800);
	Delay_ms(400);
	Arm_Turn(300);
	Delay_ms(200);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(350);
}


//抓取圆环3号位放置料盘3号位
void grap3(void)
{
	//先松开爪子并提起来
	Servo_Write(25);
	Arm_SX(-4000);
	Delay_ms(100);
	
	//爪子到圆环3号
	Arm_Turn(1680);
	Delay_ms(300);
	Arm_QH(18300);
	Delay_ms(400);
	Arm_SX(-19400);
	Delay_ms(500);
	
	//抓
	Servo_Write(0);
	Delay_ms(500);
	
	//提起来
	Arm_SX(-7000);
	Delay_ms(450);
	
	//爪子到料盘3号
	Arm_QH(3900);
	Delay_ms(400);
	Arm_Turn(1850);
	Delay_ms(200);
	Arm_SX(-10300);
	Delay_ms(500);
	
	//放
	Servo_Write(25);
	Delay_ms(500);
	
	//爪子提起来
	Arm_SX(-4000);
	Delay_ms(350);
}







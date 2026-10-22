#include "stm32f4xx.h"
#include "tjc.h"
#include "usart.h"
#include "Serial.h"
#include "Delay.h"
#include <stdio.h>


/**
  * @brief    串口屏发送一个字节（内部函数）
  * @param    b：待发送字节
  * @note     使用 USART6，等待发送数据寄存器空闲
  */
static void TJC_SendByte(uint8_t b)
{
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(USART6, b);
}


/**
  * @brief    发送串口屏指令
  * @param    s：指令字符串，空指针不发送
  * @note     自动追加三个 0xFF 作为结束符，在主循环中调用
  */
void TJC_SendString(const char *s)
{
	if (s == 0)
	{
		return;
	}

	while (*s)
	{
		TJC_SendByte((uint8_t)*s++);
	}
	TJC_SendByte(0xFF);
	TJC_SendByte(0xFF);
	TJC_SendByte(0xFF);
}


/**
  * @brief    切换串口屏页面
  * @param    page：页面编号，范围 0~3
  * @retval   1：参数有效且已发送；0：参数无效，不等待屏幕应答
  */
uint8_t TJC_SetPage(uint8_t page)
{
	char cmd[16];

	if (page > 3U)
	{
		return 0;
	}
	sprintf(cmd, "page %u", (unsigned int)page);
	TJC_SendString(cmd);
	return 1;
}


/**
  * @brief    设置地图中单个格子的颜色
  * @param    row：行（y），col：列（x），范围均为 1~5
  * @param    color：RGB565 颜色值
  * @retval   1：参数有效且已发送；0：参数无效，不等待屏幕应答
  * @note     第 3 行第 2 列对应 g32，此函数不修改障碍物记录
  */
uint8_t TJC_SetGridColor(uint8_t row, uint8_t col, uint16_t color)
{
	char cmd[32];

	if (row < 1U || row > 5U || col < 1U || col > 5U)
	{
		return 0;
	}
	sprintf(cmd, "page1.g%u%u.bco=%u", (unsigned int)row,
	        (unsigned int)col, (unsigned int)color);
	TJC_SendString(cmd);
	return 1;
}


/**
  * @brief    统一设置地图 25 个格子的颜色
  * @param    color：RGB565 颜色值
  * @note     会覆盖原有黄色格子，不修改障碍物记录
  */
void TJC_SetAllGridColor(uint16_t color)
{
	uint8_t row, col;

	for (row = 1; row <= 5; row++)
	{
		for (col = 1; col <= 5; col++)
		{
			TJC_SetGridColor(row, col, color);
		}
	}
}


/* 地图左下角为 (x=1, y=1)，控件名称为 g[y][x]。
 * 数组下标为 [y-1][x-1]，第一行表示地图最下面一行。
 * (2,2)、(4,2)、(2,4)、(4,4) 为黄色，其余格子为浅蓝色。 */
static const uint16_t grid_original_color[5][5] =
{
	{59391U, 59391U, 59391U, 59391U, 59391U},
	{59391U, 65504U, 59391U, 65504U, 59391U},
	{59391U, 59391U, 59391U, 59391U, 59391U},
	{59391U, 65504U, 59391U, 65504U, 59391U},
	{59391U, 59391U, 59391U, 59391U, 59391U}
};

// 障碍物记录：0=无障碍物，1=有障碍物，上电默认全为 0
static uint8_t grid_obstacle[5][5] = {{0}};


/**
  * @brief    更新视觉输入的障碍物状态
  * @param    x：横坐标，y：纵坐标，范围均为 1~5
  * @param    present：1=有障碍物并涂黑，0=无障碍物并恢复原色
  * @retval   1：参数有效；0：参数无效，不等待屏幕应答
  * @note     主循环中调用，状态未变化时不重复发送指令
  */
uint8_t TJC_UpdateObstacle(uint8_t x, uint8_t y, uint8_t present)
{
	uint16_t color;

	if (x < 1U || x > 5U || y < 1U || y > 5U || present > 1U)
	{
		return 0;
	}
	if (grid_obstacle[y - 1U][x - 1U] == present)
	{
		return 1;
	}

	color = present ? TJC_COLOR_BLACK : grid_original_color[y - 1U][x - 1U];
	TJC_SetGridColor(y, x, color);
	grid_obstacle[y - 1U][x - 1U] = present;
	return 1;
}


/**
  * @brief    查询单片机记录的障碍物状态
  * @param    x：横坐标，y：纵坐标，范围均为 1~5
  * @retval   0：无障碍物；1：有障碍物；-1：坐标无效
  */
int8_t TJC_IsObstacle(uint8_t x, uint8_t y)
{
	if (x < 1U || x > 5U || y < 1U || y > 5U)
	{
		return -1;
	}
	return (int8_t)grid_obstacle[y - 1U][x - 1U];
}


/**
  * @brief    清除已记录的障碍物
  * @note     已涂黑的格子恢复原色，其他格子不发送指令
  */
void TJC_ClearObstacles(void)
{
	uint8_t x, y;

	for (y = 1; y <= 5; y++)
	{
		for (x = 1; x <= 5; x++)
		{
			TJC_UpdateObstacle(x, y, 0);
		}
	}
}


/**
  * @brief    将任务码分两行显示在 page2 的全局文本控件 t0、t1
  * @param    code：仅含数字和加号的字符串，最多 23 字符
  * @note     第一行最多 7 字符，第二行最多 15 字符
  *           例如 123+321+456+654 显示为 123+321 和 456+654
  *           空指针或非法输入不发送指令，空字符串清空两行
  */
void TJC_Show_TaskCode(const char *code)
{
	char cmd[48];
	char part0[8] = {0};
	char part1[16] = {0};
	unsigned int len = 0, i, offset;

	if (code == 0)
	{
		return;
	}
	while (code[len])
	{
		if ((code[len] < '0' || code[len] > '9') && code[len] != '+')
		{
			return;
		}
		if (++len > 23U)
		{
			return;
		}
	}

	for (i = 0; i < len && i < 7U; i++)
	{
		part0[i] = code[i];
	}
	offset = (len > 7U && code[7] == '+') ? 8U : 7U;
	for (i = 0; offset + i < len && i < sizeof(part1) - 1U; i++)
	{
		part1[i] = code[offset + i];
	}

	sprintf(cmd, "page2.t0.txt=\"%s\"", part0);
	TJC_SendString(cmd);
	sprintf(cmd, "page2.t1.txt=\"%s\"", part1);
	TJC_SendString(cmd);
}


/**
  * @brief    检查并取出指定启动事件（内部函数）
  * @param    expected_side：L/R；expected_mode：1/2
  * @retval   1：匹配并消费事件，向调试串口打印；0：当前无匹配事件
  * @note     不匹配的事件暂存，等待后续接口检查，不重复消费
  *           主循环需轮询全部四个接收接口，避免暂存事件阻塞后续事件
  */
static uint8_t TJC_CheckStartKey(char expected_side, uint8_t expected_mode)
{
	static uint8_t pending = 0;
	static char side;
	static uint8_t mode;
	char log[64];

	if (pending == 0)
	{
		if (TJC_GetStartKey(&side, &mode) == 0)
		{
			return 0;
		}
		pending = 1;
	}
	if (side != expected_side || mode != expected_mode)
	{
		return 0;
	}

	pending = 0;
	sprintf(log, "TJC RX: start%c%u, side=%c, mode=%u\r\n",
			side, (unsigned int)mode, side, (unsigned int)mode);
	Serial_PrintStr(log);
	return 1;
}

/* 检查并取出startL1，收到返回1，否则返回0；在主循环调用。 */
uint8_t TJC_ReceiveStartL1(void)
{
	return TJC_CheckStartKey('L', 1);
}

/* 检查并取出startL2，收到返回1，否则返回0；在主循环调用。 */
uint8_t TJC_ReceiveStartL2(void)
{
	return TJC_CheckStartKey('L', 2);
}

/* 检查并取出startR1，收到返回1，否则返回0；在主循环调用。 */
uint8_t TJC_ReceiveStartR1(void)
{
	return TJC_CheckStartKey('R', 1);
}

/* 检查并取出startR2，收到返回1，否则返回0；在主循环调用。 */
uint8_t TJC_ReceiveStartR2(void)
{
	return TJC_CheckStartKey('R', 2);
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

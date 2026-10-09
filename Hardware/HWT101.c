/**
  ******************************************************************************
  * @file    HWT101.c
  * @brief   HWT101(维特智能 单轴陀螺仪) 串口驱动实现
  * @note    思路与工程里其它模块保持一致: 中断收数据 + 主循环非阻塞取数据。
  *          USART3_IRQHandler 里只做"收字节 + 状态机攒帧", 解析完一帧就地换算成
  *          角度/角速度写进全局变量, 主循环随时用 HWT101_GetYaw() 取即可。
  *
  *          上电配置(清零 / 设输出频率)由 HWT101_ConfigPoll() 用时间轴推进,
  *          全程非阻塞, 不会像 Delay_ms 那样卡住主循环和 OLED。
  ******************************************************************************
  */
#include "HWT101.h"
#include "Serial.h"
#include "Timer.h"		/* 用 GetTick() 做非阻塞计时 */

/* ===================== 引脚 / 串口配置(要改就改这一段) ===================== */
#define HWT101_GPIO_PORT		GPIOD
#define HWT101_GPIO_CLK			RCC_AHB1Periph_GPIOD
#define HWT101_TX_PIN			GPIO_Pin_8		/* PD8 = USART3_TX -> 陀螺仪 RX */
#define HWT101_TX_PINSOURCE		GPIO_PinSource8
#define HWT101_RX_PIN			GPIO_Pin_9		/* PD9 = USART3_RX <- 陀螺仪 TX */
#define HWT101_RX_PINSOURCE		GPIO_PinSource9
#define HWT101_GPIO_AF			GPIO_AF_USART3	/* USART3 在 F4 上是 AF7 */

#define HWT101_USART			USART3
#define HWT101_USART_CLK		RCC_APB1Periph_USART3
#define HWT101_USART_IRQn		USART3_IRQn

/* 波特率: 必须和模块里保存的值一致, 否则收到的全是乱码。
   出厂默认 9600, 改成 115200 用 FF AA 04 06 00(需解锁+保存)。 */
#define HWT101_BAUDRATE			115200

/* ===================== 上电自动清零方式 =====================
 *  1 = 软件偏置法(默认, 推荐): 不碰模块, 只把上电时那个朝向记成 0 度。
 *      优点: 无通信风险, 立即生效, 不动模块里的设置, 可随时 HWT101_ZeroHere() 重设。
 *  2 = 模块级清零: 用官方 CALSW 寄存器把模块自己的航向角置 0。
 *      优点: 模块帧里发出来的 yaw 本身就是 0 起。
 *  0 = 不清零: 直接用模块保存的绝对朝向。
 *
 *  	无论哪种方式, 上电后到清零完成前(约 1.2s)车必须静止不动, 否则零点会带上初速度误差。
 * ======================================================== */
#define HWT101_ZERO_MODE		1

/* ===================== 上电配置流程 =====================
 * 官方寄存器说明(维特智能标准通信协议):
 *   0x00 SAVE  : 0x0000=保存(不重启), 0x00FF=重启, 0x0001=恢复出厂
 *   0x01 CALSW : 0x04 = 航向角置零
 *   0x03 RRATE : 0x09=100Hz, 0x0B=200Hz, 0x06=10Hz(出厂默认)
 * 注意: 改设置前要先发解锁指令 FF AA 69 88 B5, 否则模块会拒收(超过10秒未操作也会自动上锁)。
 * ==================================================== */
#define HWT101_CFG_UNLOCK		{ 0xFF, 0xAA, 0x69, 0x88, 0xB5 }	/* 解锁寄存器 */
#if   (HWT101_ZERO_MODE == 2)
  #define HWT101_CFG_ZERO		{ 0xFF, 0xAA, 0x01, 0x04, 0x00 }	/* 航向角置零(官方 CALSW=0x04) */
  #define HWT101_HAS_ZERO_CMD	1
#else
  /* 用不到清零指令, 这里故意留一条不会执行的无效指令, 万一被误发也只会让模块忽略 */
  #define HWT101_CFG_ZERO		{ 0x00, 0x00, 0x00, 0x00, 0x00 }	/* 占位, 不会被发出 */
  #define HWT101_HAS_ZERO_CMD	0
#endif

/* 输出频率配置: 1=上电时改成 100Hz, 0=不动模块里保存的频率 */
#define HWT101_SET_RATE			1
#define HWT101_CFG_RATE			{ 0xFF, 0xAA, 0x03, 0x09, 0x00 }	/* RRATE=0x09 -> 100Hz */

/* 是否把配置写进模块内部(掉电保存)。默认 0: 不保存, 每次上电重新配一遍。
   好处是绝不写坏模块的出厂/上次保存值, 而且不用发重启指令, 不丢数据。 */
#define HWT101_CFG_PERSIST		0

/* 上电后等多久开始发配置指令(ms)。模块启动时间最大 1000ms, 留点余量。 */
#define HWT101_CFG_START_DELAY	1200
/* 每条指令之间的间隔(ms)。11 字节 @115200 只要约 1ms, 100ms 绰绰有余。 */
#define HWT101_CFG_STEP_GAP		100

/* ===================== 协议常量 ===================== */
#define HWT101_FRAME_LEN		11		/* 维特标准帧固定 11 字节 */
#define HWT101_HEADER			0x55	/* 帧头 */

/* ===================== 量程换算系数 =====================
 * 角度: raw(int16) / 32768.0 * 180.0 -> 即 180/32768 度/LSB
 * 角速度: raw(int16) / 32768.0 * 2000.0
 * 官方寄存器文档已确认"陀螺量程固定 2000°/s, 不可设置", 所以 2000 是正确系数。
 * 注意: 模块物理量程只有 ±400°/s, 超过会饱和, 但换算系数仍是 2000。 */
#define HWT101_YAW_LSB		(180.0f / 32768.0f)
#define HWT101_GYRO_FS		2000.0f
#define HWT101_GYRO_LSB		(HWT101_GYRO_FS / 32768.0f)

/* ===================== 全局数据 ===================== */
volatile HWT101_Data_t g_hwt101 = { 0.0f, 0.0f, 0u, 0u };

/* 软件零点: 上电/手动清零那一刻的模块原始 yaw */
static volatile float g_yaw_zero = 0.0f;
static volatile uint8_t g_yaw_valid = 0u;
/* 配置流程状态: 0=配置完毕 1=已发解锁 2=已发清零 3=已发设速率 4=(可选)已发保存 */
static volatile uint8_t g_cfg_stage = 0;
static uint32_t g_cfg_t0 = 0;

/* ===================== 小工具 ===================== */
/* 把角度折到 -180 ~ +180, 避免减完零点后跑到 200° 这种超范围值 */
static float HWT101_Wrap180(float a)
{
	while (a > 180.0f)
	{
		a -= 360.0f;
	}
	while (a < -180.0f)
	{
		a += 360.0f;
	}
	return a;
}

/* ===================== 帧解析状态机 ===================== */
static volatile uint8_t g_buf[HWT101_FRAME_LEN];	/* 攒帧缓冲 */
static volatile uint8_t g_idx  = 0;					/* 已收到的字节数 */
static volatile uint8_t g_sync = 0;					/* 0=在找帧头, 1=正在攒一帧 */

/* 累加和校验: 前 10 个字节求和取低 8 位, 和帧尾第 11 字节比较 */
static uint8_t HWT101_Checksum(const uint8_t *d, uint8_t len)
{
	uint8_t i, sum = 0;
	for (i = 0; i < len; i++)
	{
		sum += d[i];
	}
	return sum;
}

/* 解析一帧(11 字节, 已通过校验)。
 * 角度帧: data[6]|data[7]<<8 是 Yaw, data[4]|data[5]<<8 是 Roll(本工程不用)
 * 角速度帧: data[4]|data[5]<<8 是 ωy(本工程不用), data[6]|data[7]<<8 是 ωz */
static void HWT101_ParseFrame(const uint8_t *data)
{
	/* 先做有符号扩展再拼装, 这样负角度/负角速度才不会算错 */
	int16_t raw = (int16_t)(((uint16_t)data[7] << 8) | data[6]);

	switch (data[1])
	{
		case 0x53:	/* 角度帧 */
			g_hwt101.yaw = HWT101_Wrap180((float)raw * HWT101_YAW_LSB - g_yaw_zero);
			g_yaw_valid = 1u;
			break;

		case 0x52:	/* 角速度帧 */
			g_hwt101.gyro_z = (float)raw * HWT101_GYRO_LSB;
			break;

		default:	/* 其它帧(0x51 加速度 / 0x54 磁场等)暂时不用 */
			return;
	}

	g_hwt101.cnt++;
	g_hwt101.valid = 1u;
}

/* 状态机吃一个字节 */
static void HWT101_FeedByte(uint8_t byte)
{
	if (g_sync == 0)
	{
		/* 找帧头 */
		if (byte == HWT101_HEADER)
		{
			g_buf[0] = byte;
			g_idx    = 1;
			g_sync   = 1;
		}
		return;
	}

	g_buf[g_idx++] = byte;

	if (g_idx < 2)
	{
		return;
	}

	/* 第 2 个字节必须是合法类型字, 否则这一帧作废重新同步 */
	if (g_idx == 2 && byte != 0x53 && byte != 0x52 && byte != 0x51 && byte != 0x54)
	{
		if (byte == HWT101_HEADER)
		{
			g_buf[0] = byte;	/* 这个 0x55 当作新帧头, 不丢 */
			g_idx    = 1;
		}
		else
		{
			g_idx  = 0;
			g_sync = 0;
		}
		return;
	}

	if (g_idx >= HWT101_FRAME_LEN)
	{
		if (HWT101_Checksum((const uint8_t *)g_buf, HWT101_FRAME_LEN - 1) == g_buf[HWT101_FRAME_LEN - 1])
		{
			HWT101_ParseFrame((const uint8_t *)g_buf);
		}
		/* 校验不过就整帧丢弃(干扰/波特率不对时常见), 下一帧重新同步 */
		g_idx  = 0;
		g_sync = 0;
	}
}

/* ===================== 初始化 ===================== */
void HWT101_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* 解析状态清零 */
	g_idx  = 0;
	g_sync = 0;
	g_yaw_valid = 0u;

	/* 时钟: GPIOD 在 AHB1, USART3 在 APB1(F1 是 APB2, 别照抄 F1 例程) */
	RCC_AHB1PeriphClockCmd(HWT101_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(HWT101_USART_CLK, ENABLE);

	/* PD8 -> USART3_TX: 复用推挽输出, 要选对 AF7 */
	GPIO_PinAFConfig(HWT101_GPIO_PORT, HWT101_TX_PINSOURCE, HWT101_GPIO_AF);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = HWT101_TX_PIN;
	GPIO_Init(HWT101_GPIO_PORT, &GPIO_InitStructure);

	/* PD9 -> USART3_RX: 复用输入 */
	GPIO_PinAFConfig(HWT101_GPIO_PORT, HWT101_RX_PINSOURCE, HWT101_GPIO_AF);
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = HWT101_RX_PIN;
	GPIO_Init(HWT101_GPIO_PORT, &GPIO_InitStructure);

	/* USART3: 115200 8N1, 收+发(要发配置指令, 所以 TX 也必须开) */
	USART_InitStructure.USART_BaudRate            = HWT101_BAUDRATE;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(HWT101_USART, &USART_InitStructure);

	/* 打开接收中断 */
	USART_ITConfig(HWT101_USART, USART_IT_RXNE, ENABLE);

	/* NVIC: 和 Timer.c 保持一致用分组 2
	   优先级: 抢占 1 / 响应 1。数值比 TIM6 的 0 低, 1ms 计时不会被串口打断。 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel                   = HWT101_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(HWT101_USART, ENABLE);

	/* 启动上电配置流程的时间轴(真正发指令在 HWT101_ConfigPoll 里) */
	g_cfg_stage = 1;
	g_cfg_t0    = GetTick();
}

/* ===================== 发指令 / 上电配置流程 ===================== */
static void HWT101_SendCmd(const uint8_t *cmd, uint8_t len)
{
	uint8_t i;
	for (i = 0; i < len; i++)
	{
		USART_SendData(HWT101_USART, cmd[i]);
		while (USART_GetFlagStatus(HWT101_USART, USART_FLAG_TXE) == RESET);
	}
	/* 等最后一个字节真正发完, 免得马上发下一条时把移位寄存器里的内容冲掉 */
	while (USART_GetFlagStatus(HWT101_USART, USART_FLAG_TC) == RESET);
}

/* 放主循环里调用: 按时间轴把解锁/清零/设速率依次发出去, 全程不阻塞 */
void HWT101_ConfigPoll(void)
{
	static const uint8_t unlock_cmd[5] = HWT101_CFG_UNLOCK;
#if (HWT101_HAS_ZERO_CMD == 1)
	static const uint8_t zero_cmd[5]   = HWT101_CFG_ZERO;
#endif
#if (HWT101_SET_RATE == 1)
	static const uint8_t rate_cmd[5]   = HWT101_CFG_RATE;
#endif
#if (HWT101_CFG_PERSIST == 1)
	static const uint8_t save_cmd[5]   = { 0xFF, 0xAA, 0x00, 0x00, 0x00 };	/* 保存, 不重启 */
#endif

	if (g_cfg_stage == 0)
	{
		return;		/* 已经配完了, 直接返回 */
	}

	/* 第 1 步之前要等模块启动(START_DELAY); 后续各步之间只隔 STEP_GAP。
	   注意: 延时判断不能统一用 START_DELAY, 否则每步都要等 1.2s。 */
	{
		uint32_t wait = (g_cfg_stage == 1) ? HWT101_CFG_START_DELAY : HWT101_CFG_STEP_GAP;
		if (GetTick() - g_cfg_t0 < wait)
		{
			return;		/* 还没到时间 */
		}
	}

	switch (g_cfg_stage)
	{
		case 1:		/* 第 1 步: 解锁寄存器 */
			HWT101_SendCmd(unlock_cmd, 5);
			g_cfg_stage = 2;
			g_cfg_t0    = GetTick();
			break;

		case 2:		/* 第 2 步(解锁后 STEP_GAP): 清零 */
#if (HWT101_ZERO_MODE == 1)
			/* 软件偏置法: 必须等到模块真正吐出角度帧再记零点,
			   否则(模块上电比单片机晚等情形)会把 0 记成零点, 起步 Yaw 就不是 0 */
			if (!g_yaw_valid)
			{
				return;		/* 还没有有效角度帧, 停在这一步等数据, 时间轴不推进 */
			}
			HWT101_ZeroHere();
#elif (HWT101_ZERO_MODE == 2)
			HWT101_SendCmd(zero_cmd, 5);
			/* 模块收到清零指令后要几十 ms 才把内部航向角真正归零,
			   这里顺手把软件零点也清掉, 让读数是模块的绝对朝向 */
			HWT101_ClearZero();
#endif
			g_cfg_stage = 3;
			g_cfg_t0    = GetTick();
			break;

		case 3:
#if (HWT101_SET_RATE == 1)
			HWT101_SendCmd(rate_cmd, 5);
#endif
			g_cfg_stage = 4;
			g_cfg_t0    = GetTick();
			break;

		case 4:
#if (HWT101_CFG_PERSIST == 1)
			HWT101_SendCmd(save_cmd, 5);
#endif
			g_cfg_stage = 0;	/* 配置流程结束 */
			break;

		default:
			g_cfg_stage = 0;
			break;
	}
}

uint8_t HWT101_ConfigBusy(void)
{
	return (g_cfg_stage != 0) ? 1u : 0u;
}

/* ===================== 清零 ===================== */
/* 把"当前朝向"记成 0 度。必须在车静止时调用。 */
void HWT101_ZeroHere(void)
{
	__disable_irq();
	/* g_hwt101.yaw 是"模块原始 yaw - g_yaw_zero", 加回去就还原成模块原始 yaw,
	   再记成新零点。这样反复调用不会累积误差。 */
	g_yaw_zero   = g_hwt101.yaw + g_yaw_zero;
	g_hwt101.yaw = 0.0f;				/* 立刻显示成 0, 不用等下一帧 */
	__enable_irq();
}

/* 取消软件零点, 回到模块的绝对朝向 */
void HWT101_ClearZero(void)
{
	__disable_irq();
	/* 当前 yaw 是"原始 yaw - 零点", 要加回零点才是模块的绝对朝向 */
	g_hwt101.yaw = HWT101_Wrap180(g_hwt101.yaw + g_yaw_zero);
	g_yaw_zero   = 0.0f;
	__enable_irq();
}

/* ===================== 数据读取 ===================== */
/* 关中断做整体拷贝: 保证 yaw / gyro_z / cnt 是同一帧时间点上的值, 不会读到"半新半旧" */
uint8_t HWT101_Snapshot(HWT101_Data_t *out)
{
	__disable_irq();
	out->yaw    = g_hwt101.yaw;
	out->gyro_z = g_hwt101.gyro_z;
	out->cnt    = g_hwt101.cnt;
	out->valid  = g_hwt101.valid;
	__enable_irq();

	return out->valid;
}

float HWT101_GetYaw(void)
{
	return g_hwt101.yaw;
}

float HWT101_GetGyroZ(void)
{
	return g_hwt101.gyro_z;
}

uint32_t HWT101_GetCount(void)
{
	return g_hwt101.cnt;
}

uint8_t HWT101_IsOnline(void)
{
	return g_hwt101.valid;
}

/* ===================== 接收中断 ===================== */
/* 放在本模块里而不是 stm32f4xx_it.c, 和 Timer.c 里写 TIM6_DAC_IRQHandler 的风格一致 */
/* Print the latest snapshot when the serial command requests it. */
void HWT101_ReadAndPrint(void)
{
    HWT101_Data_t data;

    if (HWT101_ConfigBusy())
    {
        Serial_PrintStr("HWT101 configuring\r\n");
        return;
    }
    if (!HWT101_Snapshot(&data))
    {
        Serial_PrintStr("HWT101 no data\r\n");
        return;
    }

    Serial_PrintStr("HWT101 yaw=");
    Serial_PrintFloat(data.yaw, 2);
    Serial_PrintStr(" deg, gyro_z=");
    Serial_PrintFloat(data.gyro_z, 2);
    Serial_PrintStr(" deg/s\r\n");
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(HWT101_USART, USART_IT_RXNE) != RESET)
	{
		uint8_t byte = (uint8_t)(USART_ReceiveData(HWT101_USART) & 0x00FF);
		/* 读 DR 已经同时清掉了 RXNE 和 ORE */
		HWT101_FeedByte(byte);
	}

	/* 溢出错误(主循环被长时间占用时可能出现): 清掉标志, 状态机自己会重新找帧头 */
	if (USART_GetFlagStatus(HWT101_USART, USART_FLAG_ORE) != RESET)
	{
		(void)USART_ReceiveData(HWT101_USART);
	}
}

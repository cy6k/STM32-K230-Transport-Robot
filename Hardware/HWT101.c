/**
  ******************************************************************************
  * @file    HWT101.c
  * @brief   HWT101(维特智能 单轴陀螺仪) 串口驱动实现
  * @note    思路与工程里其它模块保持一致: 中断收数据 + 主循环非阻塞取数据。
  *          USART3_IRQHandler 里只做"收字节 + 状态机攒帧", 解析完一帧就地换算成
  *          角度/角速度写进全局变量, 主循环随时用 HWT101_Snapshot() 取即可。
  *
  *          HWT101_ConfigPoll() 以非阻塞方式推进模块配置及稳定基准采集。
  *          初始化阶段调用 HWT_Read_Angle()，阻塞等待角度稳定并记录零点。
  ******************************************************************************
  */
#include "HWT101.h"
#include "Serial.h"
#include "Delay.h"
#include "Motor.h"
#include "Emm_V5.h"
#include "Timer.h"		/* 用 GetTick() 做非阻塞计时 */

#define Adjust_range 0.1f		//Adjust()校准误差允许范围(度)


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

/* 上电角度基准：必须在 main 中其他初始化全部完成后开始采集。
 * 每隔 200ms 取一次最新有效角度，保留最近连续 10 次读数。
 * 这 10 次读数的最大值与最小值之差不超过 0.03 度时，取平均值作为固定零点。
 * 后续所有角度读取接口统一返回“原始角度 - 固定零点”。
 */
#define HWT101_REF_INTERVAL_MS       200U
#define HWT101_REF_SAMPLE_COUNT       10U
#define HWT101_REF_MAX_SPAN         0.03f

/* ===================== 上电配置流程 =====================
 * 官方寄存器说明(维特智能标准通信协议):
 *   0x00 SAVE  : 0x0000=保存(不重启), 0x00FF=重启, 0x0001=恢复出厂
 *   0x01 CALSW : 0x04 = 航向角置零
 *   0x03 RRATE : 0x09=100Hz, 0x0B=200Hz, 0x06=10Hz(出厂默认)
 * 注意: 改设置前要先发解锁指令 FF AA 69 88 B5, 否则模块会拒收(超过10秒未操作也会自动上锁)。
 * ==================================================== */
#define HWT101_CFG_UNLOCK		{ 0xFF, 0xAA, 0x69, 0x88, 0xB5 }	/* 解锁寄存器 */
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
static volatile float g_yaw_raw = 0.0f;
/* 基准采集状态：0=未开始，1=采集中，2=已确认；仅由主程序读写。 */
static uint8_t g_ref_state = 0;
static uint8_t g_ref_count = 0, g_ref_next = 0;
static uint32_t g_ref_sample_tick = 0;
static float g_ref_samples[HWT101_REF_SAMPLE_COUNT];
static void HWT101_ReferencePoll(void);
static volatile uint8_t g_yaw_valid = 0u;
static volatile uint32_t g_yaw_tick = 0u; /* 最近一次有效角度帧的接收时间，单位为毫秒。 */
/* 配置阶段：0=配置完成，1=等待启动并解锁，2=设置回传频率，3=结束配置。 */
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
			g_yaw_raw = (float)raw * HWT101_YAW_LSB;
			g_hwt101.yaw = HWT101_Wrap180(g_yaw_raw - g_yaw_zero);
			g_yaw_tick = GetTick();
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
	g_yaw_tick = 0u;
	g_yaw_zero = g_yaw_raw = 0.0f;
	g_hwt101.yaw = g_hwt101.gyro_z = 0.0f;
	g_hwt101.cnt = 0u;
	g_hwt101.valid = 0u;
	g_ref_state = g_ref_count = g_ref_next = 0;

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

/* 仅在主程序调用：先完成回传频率配置，再轮询采集稳定的角度基准。 */
void HWT101_ConfigPoll(void)
{
    static const uint8_t unlock_cmd[5] = HWT101_CFG_UNLOCK;
#if (HWT101_SET_RATE == 1)
    static const uint8_t rate_cmd[5] = HWT101_CFG_RATE;
#endif
#if (HWT101_CFG_PERSIST == 1)
    static const uint8_t save_cmd[5] = { 0xFF, 0xAA, 0x00, 0x00, 0x00 };
#endif
    uint32_t wait;

    if (g_cfg_stage == 0)
    {
        HWT101_ReferencePoll();
        return;
    }
    wait = (g_cfg_stage == 1) ? HWT101_CFG_START_DELAY : HWT101_CFG_STEP_GAP;
    if ((uint32_t)(GetTick() - g_cfg_t0) < wait) return;

    switch (g_cfg_stage)
    {
        case 1:
            HWT101_SendCmd(unlock_cmd, 5);
            g_cfg_stage = 2;
            break;
        case 2:
#if (HWT101_SET_RATE == 1)
            HWT101_SendCmd(rate_cmd, 5);
#endif
            g_cfg_stage = 3;
            break;
        case 3:
#if (HWT101_CFG_PERSIST == 1)
            HWT101_SendCmd(save_cmd, 5);
#endif
            g_cfg_stage = 0;
            /* 配置结束后重新计时，再等 200ms 采集第一个基准样本。 */
            g_ref_sample_tick = GetTick();
            break;
        default:
            g_cfg_stage = 0;
            break;
    }
    g_cfg_t0 = GetTick();
}

/* 初始化阶段调用一次，必须先完成 Timer_Init()、HWT101_Init() 并保持中断开启。
 * 在其他初始化结束、车身静止后调用；每 200ms 采样一次。
 * 连续 10 次读数极差不超过 0.03 度时，内部记录平均角度为固定零点并返回。
 * 未收到新鲜角度或读数不稳定时持续等待；后续角度接口自动扣除该零点。
 */
void HWT_Read_Angle(void)
{
	Serial_PrintStr("HWT101: keep still, collecting reference...\r\n");
	HWT101_ReferenceStart();
	while (HWT101_ConfigBusy())
	{
		HWT101_ConfigPoll();
		Delay_ms(1);
	}
	Serial_PrintStr("HWT101 reference=");
	Serial_PrintFloat(HWT101_GetReference(), 3);
	Serial_PrintStr(" deg, ready\r\n");
}

/* 其他初始化完成后调用一次：清除旧偏移，并开始新一轮稳定基准采集。 */
void HWT101_ReferenceStart(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_yaw_zero = 0.0f;
    g_hwt101.yaw = g_yaw_raw;
    __set_PRIMASK(primask);
    g_ref_count = g_ref_next = 0;
    g_ref_sample_tick = GetTick();
    g_ref_state = 1;
}

/* 每 200ms 检查一次最新角度；确认成功后固定基准，不再随车身转动更新。 */
static void HWT101_ReferencePoll(void)
{
    uint32_t now, frame_tick, elapsed, primask;
    float yaw, anchor, delta, minimum, maximum, sum;
    uint8_t i;

    if (g_ref_state != 1) return;
    now = GetTick();
    elapsed = (uint32_t)(now - g_ref_sample_tick);
    if (elapsed < HWT101_REF_INTERVAL_MS) return;
    i = HWT101_YawSnapshot(&yaw, &frame_tick);
    now = GetTick(); /* 快照读取后重新取时，避免接收时间比本次判断时间更新。 */
    elapsed = (uint32_t)(now - g_ref_sample_tick);
    g_ref_sample_tick = now; /* 不补采遗漏周期，避免将同一帧重复计数。 */

    if (!i || (uint32_t)(now - frame_tick) > HWT101_REF_INTERVAL_MS ||
        (uint32_t)(now - frame_tick) >= elapsed)
    {
        /* 无角度数据、数据过期或本周期没有新角度帧时，清空连续采样记录。 */
        g_ref_count = g_ref_next = 0;
        return;
    }
    g_ref_samples[g_ref_next] = yaw;
    g_ref_next = (uint8_t)((g_ref_next + 1U) % HWT101_REF_SAMPLE_COUNT);
    if (g_ref_count < HWT101_REF_SAMPLE_COUNT) g_ref_count++;
    if (g_ref_count < HWT101_REF_SAMPLE_COUNT) return;

    /* 以一个样本为参考展开角度，避免跨越 +180/-180 度时误判为大幅波动。 */
    anchor = g_ref_samples[0];
    minimum = maximum = sum = 0.0f;
    for (i = 0; i < HWT101_REF_SAMPLE_COUNT; i++)
    {
        delta = HWT101_Wrap180(g_ref_samples[i] - anchor);
        if (delta < minimum) minimum = delta;
        if (delta > maximum) maximum = delta;
        sum += delta;
    }
    /* 整组波动超标则继续采样，下一次用最新的 10 个样本重新判断。 */
    if (maximum - minimum > HWT101_REF_MAX_SPAN) return;

    primask = __get_PRIMASK();
    __disable_irq();
    g_yaw_zero = HWT101_Wrap180(anchor + sum / HWT101_REF_SAMPLE_COUNT);
    /* 用最新原始角度更新结果，包含计算基准期间由中断接收的新角度帧。 */
    g_hwt101.yaw = HWT101_Wrap180(g_yaw_raw - g_yaw_zero);
    __set_PRIMASK(primask);
    g_ref_state = 2;
}

uint8_t HWT101_ReferenceReady(void)
{
    return (g_ref_state == 2) ? 1u : 0u;
}

float HWT101_GetReference(void)
{
    return g_yaw_zero;
}

uint8_t HWT101_ConfigBusy(void)
{
    return (g_cfg_stage != 0 || !HWT101_ReferenceReady()) ? 1u : 0u;
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

/* 原子读取偏航角及其角度帧时间戳；仅收到角速度帧时不更新时间戳。 */
uint8_t HWT101_YawSnapshot(float *yaw, uint32_t *tick)
{
    uint32_t primask;
    uint8_t valid;

    if (yaw == 0 || tick == 0) return 0;
    primask = __get_PRIMASK();
    __disable_irq();
    *yaw = g_hwt101.yaw;
    *tick = g_yaw_tick;
    valid = g_yaw_valid;
    __set_PRIMASK(primask);
    return valid;
}

/* ===================== 接收中断 ===================== */
/* 放在本模块里而不是 stm32f4xx_it.c, 和 Timer.c 里写 TIM6_DAC_IRQHandler 的风格一致 */
/* 收到调试串口的读取指令后，打印最新的陀螺仪数据快照。 */
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



/* 旋转并校准到相对上电稳定零点的目标角度，范围 -180～180 度。
 * 先按实时角度计算最短转向；偏差大于5度时一次粗转，再相对细调至指定的 ±angle_range 度。
 * 例如当前180度、目标-90度，误差折算为+90度，不会反向绕转270度。
 * 粗转、细调均按实际脉冲计算理论延时，不加余量；随后读取新角度帧。
 */
void Adjust(float target_angle)
{
    Adjust_Range(target_angle, Adjust_range);
}

/* 按调用方指定的角度误差范围执行转向校准。 */
void Adjust_Range(float target_angle, float angle_range)
{
    uint32_t start, frame_tick, now, motion_tick = 0;
    float yaw, error;
    long pulses;
    uint8_t addr, coarse = 1, need_fresh = 0, yaw_valid;

    if (!(target_angle >= -180.0f && target_angle <= 180.0f))
    {
        Serial_PrintStr("Adjust: target must be -180..180\r\n");
        return;
    }
    if (!(angle_range > 0.0f && angle_range <= 180.0f))
    {
        Serial_PrintStr("Adjust: range must be >0..180\r\n");
        return;
    }
    if (HWT101_ConfigBusy())
    {
        Serial_PrintStr("Adjust: gyro reference not ready\r\n");
        return;
    }

    start = GetTick();
    while (1)
    {
        if ((uint32_t)(GetTick() - start) >= 20000U)
        {
            Serial_PrintStr("Adjust: timeout\r\n");
            break;
        }
        yaw_valid = HWT101_YawSnapshot(&yaw, &frame_tick);
        now = GetTick();
        if (!yaw_valid || (uint32_t)(now - frame_tick) > 200U ||
            (need_fresh && (int32_t)(frame_tick - motion_tick) <= 0))
        {
            /* 运动延时后最多等200ms接收新角度，不用转动期间留下的旧帧继续纠偏。 */
            if (need_fresh && (uint32_t)(now - motion_tick) <= 200U)
            {
                Delay_ms(1);
                continue;
            }
            Serial_PrintStr("Adjust: no fresh yaw\r\n");
            break;
        }
        need_fresh = 0;

        /* yaw已减去上电基准；将目标与实时角度的差折算为最短的 ±180 度。 */
        error = HWT101_Wrap180(target_angle - yaw);
        if (error >= -angle_range && error <= angle_range)
        {
            Serial_PrintStr("Adjust OK\r\n");
            break;
        }

        if (coarse && (error > 5.0f || error < -5.0f))
        {
            /* 大角度只粗转一次：按实测约4125脉冲/90度换算本次相对位移。 */
            pulses = (long)(error * (4125.0f / 90.0f));
        }
        else
        {
            /* 剩余偏差按70%细调，每步最多约10度，避免大幅来回摆动。 */
            pulses = (long)(error * (4125.0f / 90.0f) * 0.7f);
            if (pulses > 458) pulses = 458;
            if (pulses < -458) pulses = -458;
        }
        coarse = 0;

        Move_Turn(pulses);
        Delay_ms(Move_TimeMs(pulses));
        motion_tick = GetTick();
        need_fresh = 1;
    }

    /* 成功、角度异常或超时退出时，只停止底盘四轮，不影响机械臂。 */
    for (addr = 1; addr <= 4; addr++)
    {
        Emm_V5_Stop_Now(addr, false);
        Delay_ms(1);
    }
}

#include "stm32f4xx.h"
#include "Motor.h"
#include "Emm_V5.h"
#include "Delay.h"


#define E_vel 3000
#define E_acc 150

/*
 * 底盘坐标系模型：
 *   每条命令的数值都是本次相对位移（qh / zy / turn）：
 *     w1 = qh - zy - turn
 *     w2 = qh + zy - turn
 *     w3 = qh - zy + turn
 *     w4 = qh + zy + turn
 *   换算出四轮本次位移后，以相对位置模式同步发送。
 *   例：连续发送两次a1000，每次都前进1000脉冲。
 *       a1000后b1000，先前进1000，再横移1000。
 */


/**
  * @brief    位置模式
  * @param    addr：电机地址     dir：0为CW，其余CCW
  * @param    vel ：速度(RPM)    acc ：加速度(0-255)
  * @param    clk ：脉冲数       raF ：false相对/true绝对
  * @param    snF ：多机同步标志，false不启用/true启用
  */


void EC_Enable(void)
{
	Emm_V5_En_Control(1, true, false);
	Emm_V5_En_Control(2, true, false);
	Emm_V5_En_Control(3, true, false);
	Emm_V5_En_Control(4, true, false);
}


/**
  * 函    数：单电机按相对位移运动（内部核心）
  * 参    数：addr   —— 电机地址 1~4
  *           delta  —— 本次位移(脉冲数，带符号)
  *           snF    —— false=立即执行，true=只装填等待同步触发
  * 备    注：方向已按镜像安装标定：1/2号正方向=CCW，3/4号正方向=CW
  */
static void Wheel_Move(uint8_t addr, long delta, bool snF)
{
	uint8_t dPos = (addr <= 2) ? 1 : 0;      // 正方向：1/2号=CCW，3/4号=CW
	uint8_t dNeg = (addr <= 2) ? 0 : 1;      // 反方向

	if (delta >= 0)
	{
		Emm_V5_Pos_Control(addr, dPos, E_vel, E_acc, (uint32_t)delta, 0, snF);
	}
	else
	{
		Emm_V5_Pos_Control(addr, dNeg, E_vel, E_acc, (uint32_t)(-(delta + 1)) + 1U, 0, snF);
	}
}


/**
  * 函    数：四轮同步运动（核心）
  * 参    数：t1~t4 —— 四个电机的本次相对位移(脉冲数，带符号)
  * 备    注：先依次装填(snF=1，命令间1ms防丢包)，再广播触发 00 FF 66 6B，
  *           四轮同一瞬间开始运动。装填阶段的1ms只拖慢准备动作，不影响同步性
  */
void Car_Sync(long t1, long t2, long t3, long t4)
{
	Wheel_Move(1, t1, 1);
	Delay_ms(1);
	Wheel_Move(2, t2, 1);
	Delay_ms(1);
	Wheel_Move(3, t3, 1);
	Delay_ms(1);
	Wheel_Move(4, t4, 1);
	Delay_ms(1);
	Emm_V5_Synchronous_motion(0);   // 广播地址0触发，按说明书规则只有1号回应答
}


/**
  * 函    数：将本次三方向位移换算为四轮相对位移并同步执行（内部）
  */
static void Sync_By_Delta(long qh, long zy, long turn)
{
	Car_Sync(qh - zy - turn,
	         qh + zy - turn,
	         qh - zy + turn,
	         qh + zy + turn);
}


/**
  * 函    数：底盘前后（按本次相对位移同步运动）
  * 参    数：mcs —— 本次前后位移(脉冲数)，正=前方，负=后方
  * 备    注：相对语义；连续两次a1000会前进两次，各1000脉冲
  */
void Move_QH(long mcs)
{
	Sync_By_Delta(mcs, 0, 0);
}


/**
  * 函    数：底盘左右（按本次相对位移同步运动）
  * 参    数：mcs —— 本次左右位移(脉冲数)，正负即左右
  */
void Move_ZY(long mcs)
{
	Sync_By_Delta(0, mcs, 0);
}


/**
  * 函    数：底盘原地旋转（按本次相对位移同步运动）
  * 参    数：mcs —— 本次旋转位移(脉冲数)，正负即两个方向
  */
void Move_Turn(long mcs)
{
	Sync_By_Delta(0, 0, mcs);
}




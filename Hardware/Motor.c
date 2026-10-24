#include "stm32f4xx.h"
#include "Motor.h"
#include "Emm_V5.h"
#include "Delay.h"
#include <math.h>
#include <limits.h>


#define E_vel 3000
#define E_acc 170


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
  *           vel    —— 速度(RPM)，acc为驱动器加速档位
  *           snF    —— false=立即执行，true=只装填等待同步触发
  * 备    注：方向已按镜像安装标定：1/2号正方向=CCW，3/4号正方向=CW
  */
static void Wheel_Move(uint8_t addr, long delta, uint16_t vel, uint8_t acc, bool snF)
{
	uint8_t dPos = (addr <= 2) ? 1 : 0;      // 正方向：1/2号=CCW，3/4号=CW
	uint8_t dNeg = (addr <= 2) ? 0 : 1;      // 反方向

	if (delta >= 0)
	{
		Emm_V5_Pos_Control(addr, dPos, vel, acc, (uint32_t)delta, 0, snF);
	}
	else
	{
		Emm_V5_Pos_Control(addr, dNeg, vel, acc, (uint32_t)(-(delta + 1)) + 1U, 0, snF);
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
	Wheel_Move(1, t1, E_vel, E_acc, 1);
	Delay_ms(1);
	Wheel_Move(2, t2, E_vel, E_acc, 1);
	Delay_ms(1);
	Wheel_Move(3, t3, E_vel, E_acc, 1);
	Delay_ms(1);
	Wheel_Move(4, t4, E_vel, E_acc, 1);
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
  * 参    数：mcs —— 本次左右位移(脉冲数)，正=右方，负=左方
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






/**
  * 函    数：底盘右上斜向运动
  * 参    数：mcs —— 前后、左右分量各自的脉冲数，正=指定方向，负=反方向
  * 备    注：两轮位移为0，另两轮位移为2倍mcs；保持原有速度和加速度
  */
void Move_slant1(long mcs)
{
	Sync_By_Delta(mcs, mcs, 0);
}


/**
  * 函    数：底盘右下斜向运动
  * 参    数：mcs —— 前后、左右分量各自的脉冲数，正=指定方向，负=反方向
  * 备    注：两轮位移为0，另两轮位移为2倍mcs；保持原有速度和加速度
  */
void Move_slant2(long mcs)
{
	Sync_By_Delta(-mcs, mcs, 0);
}





/* 保留原来的运动接口；_Td 只增加发令前检查和发令后到位等待。
 * 零位移不重新发命令，仍检查四轮是否到位。
 */
static Motor_Td_Result_t Move_Wait_Td(void (*move)(long), long mcs, uint8_t changedMask)
{
    Motor_Td_Result_t result;
    result = Motor_Td_Prepare(0x0F, mcs != 0 ? changedMask : 0);
    if (result != MOTOR_TD_OK) return result;
    if (mcs != 0) move(mcs);
    return Motor_Td_Wait();
}

Motor_Td_Result_t Move_QH_Td(long mcs)
{
    return Move_Wait_Td(Move_QH, mcs, 0x0F);
}

Motor_Td_Result_t Move_ZY_Td(long mcs)
{
    return Move_Wait_Td(Move_ZY, mcs, 0x0F);
}

Motor_Td_Result_t Move_Turn_Td(long mcs)
{
    return Move_Wait_Td(Move_Turn, mcs, 0x0F);
}

Motor_Td_Result_t Move_slant1_Td(long mcs)
{
    /* 斜向运动两轮为2*mcs，防止32位long计算溢出。 */
    if (mcs > 1073741823L || mcs < -1073741823L)
        return MOTOR_TD_ARGUMENT_ERROR;
    return Move_Wait_Td(Move_slant1, mcs, 0x0A);
}

Motor_Td_Result_t Move_slant2_Td(long mcs)
{
    if (mcs > 1073741823L || mcs < -1073741823L)
        return MOTOR_TD_ARGUMENT_ERROR;
    return Move_Wait_Td(Move_slant2, mcs, 0x05);
}



/**
  * 函    数：底盘任意角度平移（不旋转，不等待到位）
  * 参    数：angle —— -180~180度，0向前，正数向右偏，负数向左偏
  *           mcs   —— 本次合成位移脉冲数，负数反向，0不发令
  * 备    注：角度非法或四轮换算超出long范围时不发令；四轮按位移比分配速度和加速档位
  */
void Move(float angle, long mcs)
{
	long delta[2];
	uint32_t pulses[2], maxPulse = 0, minPulse;
	uint16_t vel[2], rampStep;
	uint8_t acc[2];
	uint8_t i, addr;
	double radians, qh, zy, value;

	if (!(angle >= -180.0f && angle <= 180.0f) || mcs == 0) return;
	radians = (double)angle * (3.14159265358979323846 / 180.0);
	qh = (double)mcs * cos(radians);
	zy = (double)mcs * sin(radians);

	// 1/3号轮为前后减左右，2/4号轮为前后加左右。
	for (i = 0; i < 2; i++)
	{
		value = (i == 0) ? qh - zy : qh + zy;
		value = (value >= 0.0) ? floor(value + 0.5) : ceil(value - 0.5);
		if (value > (double)LONG_MAX || value < (double)LONG_MIN) return;
		delta[i] = (long)value;
		pulses[i] = (delta[i] >= 0) ? (uint32_t)delta[i] : (uint32_t)(-(delta[i] + 1)) + 1U;
		if (pulses[i] > maxPulse) maxPulse = pulses[i];
	}
	if (maxPulse == 0) return;

	// Emm V5每增加1RPM需(256-acc)*50us，慢轮的加速间隔应按位移比增大。
	minPulse = (pulses[0] < pulses[1]) ? pulses[0] : pulses[1];
	if (minPulse == 0) minPulse = maxPulse;
	value = floor(255.0 * minPulse / maxPulse);
	if (value < 1.0) value = 1.0;
	rampStep = 256U - E_acc;
	// 慢轮所需间隔超出255档时，同时缩短快轮间隔，避免只限制慢轮而破坏比例。
	if (value < rampStep) rampStep = (uint16_t)value;

	for (i = 0; i < 2; i++)
	{
		vel[i] = (uint16_t)((double)E_vel * pulses[i] / maxPulse + 0.5);
		if (vel[i] == 0) vel[i] = 1;
		acc[i] = E_acc;
		if (E_acc != 0 && pulses[i] != 0)
		{
			value = floor((double)rampStep * maxPulse / pulses[i] + 0.5);
			if (value > 255.0) value = 255.0;
			acc[i] = (uint8_t)(256U - (uint16_t)value);
		}
	}

	// shortcut: 加速档位为整数，极小轮速比只能近似同步；更高轨迹精度需统一轨迹控制。
	for (addr = 1; addr <= 4; addr++)
	{
		i = (uint8_t)((addr - 1) % 2);
		Wheel_Move(addr, delta[i], vel[i], acc[i], 1);
		Delay_ms(1);
	}
	Emm_V5_Synchronous_motion(0);
}

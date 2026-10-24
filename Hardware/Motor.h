#ifndef __MOTOR_H
#define __MOTOR_H

#include "Motor_Td.h"


// 4个电机使能
void EC_Enable(void);


//四轮同步运动核心（装填+广播触发，四轮同时启动）
//参数为各轮的相对转动量(脉冲数，带符号)
void Car_Sync(long t1, long t2, long t3, long t4);


//底盘运动控制（相对量，三个动作各自独立累计，互不覆盖）
void Move_QH(long mcs);
void Move_ZY(long mcs);
void Move_Turn(long mcs);



//45度斜向运动：mcs为前后、左右各自的位移分量；正数按注释方向，负数反向
void Move_slant1(long mcs);    // 正时右上，负时左下
void Move_slant2(long mcs);    // 正时右下，负时左上



/* 阻塞到位检测版本：参数与原函数一致；成功返回 MOTOR_TD_OK。
 * 需先初始化 TIM6/USART2，并保持中断开启；仅在主程序中调用。
 * 底盘等待四轮均到位；失败时停止四轮，上层必须检查返回值。
 */
Motor_Td_Result_t Move_QH_Td(long mcs);
Motor_Td_Result_t Move_ZY_Td(long mcs);
Motor_Td_Result_t Move_Turn_Td(long mcs);
Motor_Td_Result_t Move_slant1_Td(long mcs);
Motor_Td_Result_t Move_slant2_Td(long mcs);


//任意角度平移：-180~180度，0前、90右、-90左；mcs为合成位移，负数反向
//不等待到位；0脉冲、非法角度或四轮脉冲超出long范围时不发令
void Move(float angle, long mcs);

#endif



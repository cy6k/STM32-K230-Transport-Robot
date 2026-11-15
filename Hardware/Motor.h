#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>



// 4个电机使能
void EC_Enable(void);


//四轮同步运动核心（装填+广播触发，四轮同时启动）
//参数为各轮的相对转动量(脉冲数，带符号)
void Car_Sync(long t1, long t2, long t3, long t4);


//单轴平移理论用时(ms)，按当前底盘参数和3200脉冲/圈计算，不加余量
uint32_t Move_TimeMs(long mcs);

//底盘运动控制（相对量，三个动作各自独立累计，互不覆盖）
void Move_QH(long mcs);
void Move_ZY(long mcs);
void Move_Turn(long mcs);



//45度斜向运动：mcs为前后、左右各自的位移分量；正数按注释方向，负数反向
void Move_slant1(long mcs);    // 正时右上，负时左下
void Move_slant2(long mcs);    // 正时右下，负时左上



//任意角度平移：-180~180度，0前、90右、-90左；mcs为合成位移，负数反向
//speed为最快轮速度1~5000RPM，acceleration为加速档位0~255，0为直接启动
//四轮使用指定加速档位，仅按位移比分配速度；同步启动，到位时间可不同
//不等待到位，返回四轮理论用时(ms)，不加余量；未发令返回0
uint32_t Move(float angle, long mcs, uint16_t speed, uint8_t acceleration);

#endif



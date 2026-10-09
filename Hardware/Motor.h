#ifndef __MOTOR_H
#define __MOTOR_H


// 4个电机使能
void EC_Enable(void);


//四轮同步运动核心（装填+广播触发，四轮同时启动）
//参数为各轮的相对转动量(脉冲数，带符号)
void Car_Sync(long t1, long t2, long t3, long t4);


//底盘运动控制（相对量，三个动作各自独立累计，互不覆盖）
void Move_QH(long mcs);
void Move_ZY(long mcs);
void Move_Turn(long mcs);





#endif

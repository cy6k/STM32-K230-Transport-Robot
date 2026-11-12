#ifndef __ARM_H
#define __ARM_H




//机械臂初始化
void Arm_Enable(void);


//机械臂绝对位置运动
void Arm_Turn(long mcs);
void Arm_SX(long mcs);
void Arm_QH(long mcs);



#endif


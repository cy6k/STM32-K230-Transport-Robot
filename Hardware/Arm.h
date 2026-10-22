#ifndef __ARM_H
#define __ARM_H

#include "Motor_Td.h"



//机械臂初始化
void Arm_Enable(void);

//机械臂绝对位置运动
void Arm_Turn(long mcs);
void Arm_SX(long mcs);
void Arm_QH(long mcs);



/* 阻塞到位检测版本：沿用 dis_1/2/3 软件目标坐标，分别等待5/6/7号。
 * 成功返回 MOTOR_TD_OK；失败停止该轴，调用方应结束当前流程。
 * 运动失败后软件目标不代表实际位置，须重新校准零点后再运行。
 * 同一目标重复调用只等待该轴，不再发送零位移命令打断原运动。
 */
Motor_Td_Result_t Arm_Turn_Td(long mcs);
Motor_Td_Result_t Arm_SX_Td(long mcs);
Motor_Td_Result_t Arm_QH_Td(long mcs);

#endif

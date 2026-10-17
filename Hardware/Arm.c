#include "stm32f4xx.h"
#include "Arm.h"
#include "Emm_V5.h"


//三个电机加速度速度分别设置
#define vel_1 3000
#define acc_1 220

#define vel_2 3000
#define acc_2 235

#define vel_3 3000
#define acc_3 250

long dis_1=0;
long dis_2=0;
long dis_3=0;

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */


//机械臂初始化
void Arm_Enable(void)
{
	Emm_V5_En_Control(5, true, false);
	Emm_V5_En_Control(6, true, false);
	Emm_V5_En_Control(7, true, false);
}


//底部机械臂旋转
void Arm_Turn(long mcs)
{
	 long pos = mcs - dis_1; 
    if (pos >= 0)
    {
        Emm_V5_Pos_Control(5, 1, vel_1, acc_1, (uint32_t)pos, 0, 0);
    }
    else
    {
        Emm_V5_Pos_Control(5, 0, vel_1, acc_1, (uint32_t)(-pos), 0, 0);
    }
    dis_1 = mcs;
}

//机械臂上下运动
void Arm_SX(long mcs)
{
	 long pos = mcs - dis_2; 
    if (pos >= 0)
    {
        Emm_V5_Pos_Control(6, 1, vel_2, acc_2, (uint32_t)pos, 0, 0);
    }
    else
    {
        Emm_V5_Pos_Control(6, 0, vel_2, acc_2, (uint32_t)(-pos), 0, 0);
    }
    dis_2 = mcs;
}


//机械臂前后运动
void Arm_QH(long mcs)
{
	 long pos = mcs - dis_3; 
    if (pos >= 0)
    {
        Emm_V5_Pos_Control(7, 1, vel_3, acc_3, (uint32_t)pos, 0, 0);
    }
    else
    {
        Emm_V5_Pos_Control(7, 0, vel_3, acc_3, (uint32_t)(-pos), 0, 0);
    }
    dis_3 = mcs;
}




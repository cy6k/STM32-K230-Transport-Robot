#ifndef __VISION_WORK_H
#define __VISION_WORK_H

#include "stm32f4xx.h"

#define VISION_QR_CONFIRM_FRAMES 2U /* 二维码连续确认帧数 */

/* 四组三位任务码；重进模式1清零，切换其他模式保留。 */
extern uint8_t QR_Data[4][3];

/* 料盘：[x偏差,y偏差,颜色1~6]；模式2清零，z=0表示尚无有效数据。 */
extern int32_t Material_Data[3];

/* 圆环：[x偏差,y偏差]；模式3/5清零，初始零值不代表识别成功。 */
extern int32_t Ring_Data[2];

/* 1扫码、2料盘、3圆环放料、4休息、5叠放；2/3/5开灯，4关灯，1保持。 */
void Vision_SetMode(uint8_t mode);

/* 模式1循环调用：连续确认后保存并显示；1=完成，0=未完成。 */
uint8_t Vision_Read_QR(void);

/* 模式2接收并打印最新合法(x,y,z)；1=有新数据，0=无新数据。 */
uint8_t Vision_Read_material(void);
uint8_t Vision_Read_material_now(void); /* 同上，用于定位后等待z变化。 */

/* 模式3/5接收并打印最新合法(x,y)；1=有新数据，0=无新数据。 */
uint8_t Vision_Read_Ring(void);

/* 接收函数均无完整帧即返回，错误帧不覆盖数组；使用前先初始化定时器并设置模式。 */

/* 读取圆环数据后循环调用：先左右、再前后修正，再校准至最近的90度整数倍。
 * 朝向误差分别≤orient1_angle、orient2_angle度，且之后连续3帧两轴偏差均在对应像素范围内才完成；超差则继续修正。
 * 1=完成，0=未完成；重新设置对应模式可再次定位。
 */
uint8_t orient1(void); /* 模式3：两批圆环放料前定位，像素误差用orient1_range。 */
uint8_t orient2(void); /* 模式5：叠放前定位，像素误差用orient2_range。 */

/* 模式2读取料盘数据后调用：先左右、再前后单次修正，不复检位置、不校准朝向。
 * 保留运动期间的接收帧供z变化判断；1=修正完成，0=未处理或越界；重进模式2可再次定位。
 */
uint8_t Locate(void);

#endif

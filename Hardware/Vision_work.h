#ifndef __VISION_WORK_H
#define __VISION_WORK_H

#include "stm32f4xx.h"

#define VISION_QR_CONFIRM_FRAMES 2U /* 连续收到相同二维码的确认次数 */

/* 四组三位数字，按二维码原顺序保存；每行一组，每个元素为数字0~9。
 * 只保存在RAM中；模式1重新读取前清空，切换到其它模式仍保留。
 */
extern uint8_t QR_Data[4][3];

/* 最新一帧料盘数据：[0]=x、[1]=y（带符号像素偏差），[2]=z（颜色1~6）。
 * 切换模式2时清零；z为0表示本次还没收到合法帧。后续每帧整体覆盖。
 */
extern int32_t Material_Data[3];

/* 最新一帧圆环差值：[0]=x、[1]=y，均为带符号整数。
 * 切换模式3时清零；无新帧保留旧值，初始零值不代表已经识别到圆环。
 */
extern int32_t Ring_Data[2];

void Vision_SetMode(uint8_t mode); /* 1~4对应@m1FFF~@m4FFF；模式2、3各启动一轮定位 */
void Vision_Read_QR(void);         /* 模式1：等待连续2帧相同内容，写入QR_Data后返回 */
/* 模式2：反复调用，每200ms把合法(x,y,z)记录到Material_Data后打印。
 * 无新帧立即返回；错误帧不修改数组、不打印；一周期内可依次处理多帧。
 * 先Timer_Init()、Vision_SetMode(2)；调试时在case k内循环调用，发送m退出接收。
 */
void Vision_Read_material(void);

/* 模式3：调试时在case l内循环调用，按200ms节拍处理K230每秒发送的(x,y)。
 * 合法帧整体更新Ring_Data后打印；无完整帧直接返回，错误帧不覆盖旧值。
 * 发送m退出case l的接收定位循环，并通知K230切换到休息模式。
 */
void Vision_Read_Ring(void);

/* 在Vision_Read_Ring()后持续调用；只用新帧纠偏，先x后y，到±5内结束。
 * x用Move_ZY_Td()、y用Move_QH_Td()，每次纠偏等待四轮到位后返回。
 * 定位完成后继续打印，重新切换模式3可再次定位。
 */
void orient(void);

/* 在 Vision_Read_material() 后持续调用；x、y均在±5内结束本轮定位。
 * 按200ms接收节拍使用新帧判断；每次纠偏动作内部等待四轮到位后返回。
 * 到位后继续读取、打印数据；重新切换模式2可再次定位。 */
void Locate(void);

#endif

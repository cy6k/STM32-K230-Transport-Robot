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

uint8_t Vision_Read_QR(void);      /* 非阻塞：1=已保存并显示；0=暂未确认；在case j循环调用 */
/* 模式2：反复调用，每200ms把合法(x,y,z)记录到Material_Data后打印。
 * 无新帧立即返回；错误帧不修改数组、不打印；一周期内可依次处理多帧。
 * 先Timer_Init()、Vision_SetMode(2)；调试时在case k内循环调用，发送m退出接收。
 */
/* 本次收到合法新帧返回1，无新帧返回0；原有调用可忽略返回值。 */
uint8_t Vision_Read_material(void);

/* 模式3：调试时在case l内循环调用，按200ms节拍处理K230每秒发送的(x,y)。
 * 合法帧整体更新Ring_Data后打印；无完整帧直接返回，错误帧不覆盖旧值。
 * 发送m退出case l的接收定位循环，并通知K230切换到休息模式。
 */
void Vision_Read_Ring(void);

/* 在Vision_Read_Ring()后持续调用；只用新帧纠偏，两轴均到±5内结束。
 * 先Move_ZY()、再Move_QH()，修正80%偏差，每轴最多200脉冲；不使用到位检测。
 * 每轴发令后延时260ms，两步结束后清除旧帧；重新切换模式3可再次定位。
 */
uint8_t orient(void); /* 1=本帧完成圆环定位，0=尚未完成或无新帧 */

/* 在 Vision_Read_material() 后持续调用；x、y均在±5内结束本轮定位。
 * 按200ms接收节拍使用新帧判断；先左右再前后，修正80%偏差，每轴最多200脉冲。
 * 不使用到位检测，每轴发令后延时260ms，两步结束后清除旧帧，再等新帧判断。
 * 到位后继续读取、打印数据；重新切换模式2可再次定位。 */
void Locate(void);

#endif

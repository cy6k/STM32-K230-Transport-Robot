#ifndef __TJC_H
#define __TJC_H

#include "stm32f4xx.h"


#define TJC_COLOR_BLACK       0U       // 黑色
#define TJC_COLOR_WHITE       65535U   // 白色
#define TJC_COLOR_MAP_NORMAL  59391U   // 普通格子的浅蓝色


// 发送屏幕指令，自动追加三个 0xFF 结束符，在主循环中调用
void TJC_SendString(const char *s);

// 页面切换：page 范围 0~3；返回 1=已发送，0=参数无效，不等待应答
uint8_t TJC_SetPage(uint8_t page);

// 直接设置格子颜色：行 row=y、列 col=x，范围 1~5，不修改障碍物记录
// 单格设置返回 1=已发送，0=参数无效，不等待应答
uint8_t TJC_SetGridColor(uint8_t row, uint8_t col, uint16_t color);
void TJC_SetAllGridColor(uint16_t color);

// 障碍物状态：左下角为 (1,1)，x 横向、y 纵向，范围 1~5
// present：1=涂黑，0=恢复原色；返回 1=参数有效，0=参数无效
// 在主循环中调用，状态未变化时不重复发送，不等待屏幕应答
uint8_t TJC_UpdateObstacle(uint8_t x, uint8_t y, uint8_t present);

// 查询障碍物：返回 0=无，1=有，-1=坐标无效
int8_t TJC_IsObstacle(uint8_t x, uint8_t y);

// 清除障碍物记录，恢复已涂黑格子的原色，其他格子不发送指令
void TJC_ClearObstacles(void);

// 任务码：仅含数字和加号，最多 23 字符，分两行写入 page2.t0、t1
void TJC_Show_TaskCode(const char *code);


// 启动消息接收：返回 1=收到对应事件并打印，0=当前没有对应事件
// 主循环需轮询全部四个接口，每个事件只消费一次
uint8_t TJC_ReceiveStartL1(void);
uint8_t TJC_ReceiveStartL2(void);
uint8_t TJC_ReceiveStartR1(void);
uint8_t TJC_ReceiveStartR2(void);


#endif

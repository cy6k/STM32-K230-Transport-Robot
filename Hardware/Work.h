#ifndef __WORK_H
#define __WORK_H

#include "stm32f4xx.h"


// 串口命令测试入口
void test(void);


//第一批取料：已读二维码、料盘为空；按2->1->3->2推算，每次新停留抓一个并归位；1完成，0等待/位置或归位失败
uint8_t catch_first(void);

// catch_ij：转盘i号取料，放到车载料盘j号（i、j为1~3）
void catch_11(void);
void catch_22(void);
void catch_33(void);
void catch_12(void);
void catch_13(void);
void catch_21(void);
void catch_23(void);
void catch_31(void);
void catch_32(void);



//第一批放料：已读二维码、摄像头大致对准圆环；内部先定位再放料，1完成；0定位超时退出
uint8_t put_first(void);

//车载料盘i号取料，放到圆环i号（i为1~3）
void put1(void);
void put2(void);
void put3(void);

//第一批取回：粗加工放完后调用，按二维码第二组顺序抓回同号料盘
void grap_first(void);

//圆环i号取料，放到车载料盘i号（i为1~3）
void grap1(void);
void grap2(void);
void grap3(void);


// 路线测试，_Td版本使用阻塞式到位检测
void test_run1_Td(void);
void test_run2_Td(void);


#endif 

#ifndef __WORK_H
#define __WORK_H

#include "stm32f4xx.h"


// 串口命令测试入口
void test(void);


// 路线测试：普通运动接口加固定延时，Adjust保留角度闭环
void test_run1(void);
void test_run2(void);


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


//车载料盘i号取料，放到圆环i号（i为1~3）
void put1(void);
void put2(void);
void put3(void);


//圆环i号取料，放到车载料盘i号（i为1~3）
void grap1(void);
void grap2(void);
void grap3(void);



//第一批取料：已读二维码、料盘为空；定位后首次z变化即连续抓三个，前两抓用当前位、第三抓用再换位一次的位置，不回识别姿态，1完成，0超时/位置错误
uint8_t catch_first(void);

//第二批取料：按二维码第三组颜色顺序、第四组位置取料；连续三抓逻辑同第一批，1完成，0超时/位置错误
uint8_t catch_second(void);

//第一批放料：已读二维码、摄像头大致对准圆环；内部先定位再放料，1完成；0定位超时退出
uint8_t put_first(void);

//第一批取回：粗加工放完后调用，按二维码第二组顺序抓回同号料盘
void grap_first(void);

//第二批粗加工放料：先定位，按二维码第四组顺序放到同号圆环，1完成，0定位超时
uint8_t put_second(void);

//第二批粗加工取回：按二维码第四组顺序，从圆环抓回同号车载料盘
void grap_second(void);

//第二批暂存区叠放：先定位，按第三组顺序从第四组车载位置取料，叠到第一批同色物料上；1完成，0任务错误/定位超时
uint8_t put_third(void);





#endif 




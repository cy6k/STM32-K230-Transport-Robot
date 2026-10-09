#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f4xx.h"

/* ´®¿Ú1µ÷ÊÔÄ£¿é£¨PA9=TX, PA10=RX, 115200-8-N-1£© */

void     Serial_Init(void);
uint16_t Serial_Available(void);
int16_t  Serial_Read(void);
long     Serial_ParseInt(void);

void     Serial_SendByte(uint8_t Byte);
void     Serial_PrintStr(const char *str);
void     Serial_PrintInt(long num);
void     Serial_PrintFloat(float f, uint8_t digits);
void     Serial_Println(void);

#endif

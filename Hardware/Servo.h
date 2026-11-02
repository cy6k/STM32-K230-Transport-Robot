#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx.h"

void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);
void Servo_Write(uint8_t angle);


#endif

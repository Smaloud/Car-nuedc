#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f1xx_hal.h"

// º¯ÊýÉùÃ÷
void Encoder_Init(void);
void Encoder_Init(void);
float Get_Wheel_Speed_R(void);
float Get_Wheel_Speed_L(void);
int Get_Wheel_Direction_R(void);
int Get_Wheel_Direction_L(void);
int32_t getL(void);
#endif // ENCODER_H

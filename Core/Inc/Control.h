#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"

#define A1_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET)
#define A1_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET)
#define A2_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_SET)
#define A2_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,GPIO_PIN_RESET)
#define B1_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET)
#define B1_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_RESET)
#define B2_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET)
#define B2_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET)

extern volatile uint8_t reset_flag;  
void Control_Init(void);
void Control_Loop(void);
void Set_PWM(TIM_HandleTypeDef *htim, float duty_cycle, uint32_t channel);
void Set_Velocity(TIM_HandleTypeDef *htim,uint32_t channel, float speed, float max_speed);
void Control_Loop_AngleX(void);
void Control_Loop_Angle0(void);
void Set_direction_L(uint8_t state);
void Set_direction_R(uint8_t state);
void Motor(uint16_t L, uint16_t R);
#endif // CONTROL_H

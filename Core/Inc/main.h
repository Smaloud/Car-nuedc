/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern volatile uint8_t reset_flag;  
//extern uint8_t Number1, Number2, Number3;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_6
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI9_5_IRQn
#define WK_UP_Pin GPIO_PIN_0
#define WK_UP_GPIO_Port GPIOA
#define PWM_R_Pin GPIO_PIN_1
#define PWM_R_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_6
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_7
#define BIN2_GPIO_Port GPIOA
#define HALL_SENSOR_Pin GPIO_PIN_4
#define HALL_SENSOR_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_8
#define BT1_GPIO_Port GPIOD
#define BT2_Pin GPIO_PIN_9
#define BT2_GPIO_Port GPIOD
#define Encoder_L__Pin GPIO_PIN_12
#define Encoder_L__GPIO_Port GPIOD
#define Encoder_L_D13_Pin GPIO_PIN_13
#define Encoder_L_D13_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOG
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOG
#define Encoder_R__Pin GPIO_PIN_6
#define Encoder_R__GPIO_Port GPIOC
#define Encoder_R_C7_Pin GPIO_PIN_7
#define Encoder_R_C7_GPIO_Port GPIOC
#define PWM_L_Pin GPIO_PIN_8
#define PWM_L_GPIO_Port GPIOA
#define Cam1_Pin GPIO_PIN_9
#define Cam1_GPIO_Port GPIOA
#define Cam2_Pin GPIO_PIN_10
#define Cam2_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOD
#define KEY4_Pin GPIO_PIN_5
#define KEY4_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

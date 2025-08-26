/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <math.h>
#include "openmv.h"
#include "BlueTooth.h"
#include "Control.h"
#include "Encoder.h"
#include "mpuAngle.h"
#include "OLED.h"
#include "menu.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint8_t reset_flag;  
uint16_t mpu6050Time = 0;
uint16_t OLEDTime = 0;
extern uint8_t Angle,Number2, Number3;
uint16_t encoderTime = 0;
//extern uint8_t Number1, Number2, Number3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void System_Reset(void){
//  reset_flag = 0;
//  while(!reset_flag);
//  reset_flag = 0;
//  HAL_NVIC_SystemReset();
//}


#define BUFF_MAX_SIZE 255 //最长缓冲区位数256

typedef struct usartRecvType { //openmv的uart1
    uint8_t recvData;	// 接收数据
    uint8_t recvBuff[BUFF_MAX_SIZE];  // 接收数据的缓冲区
    uint8_t recvFlag; // 成功接收一包数据的标志位
    uint16_t recvNum; // 接收数据的字节数
} usartRecvType_t;
usartRecvType_t usart1Recv = {0}; //初始化 接收结构体

uint8_t uart3_rxbuff; //蓝牙的uart3


// 串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {		
		static uint8_t temp;
		temp = usart1Recv.recvData;

        OpenMV_Data_Receive(temp);
		HAL_UART_Transmit(&huart1, &temp, 1,100);
		
        // 重新开启串口接收中断
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&usart1Recv.recvData, 1); //接收, 存到recvData里
    }
	
	if(huart->Instance == USART3)
    {		
		static uint8_t temp3;
		temp3 = usart1Recv.recvData;

        OpenMV_Data_Receive(temp3);
		HAL_UART_Transmit(&huart3, &temp3, 1,100);
		
        // 重新开启串口接收中断
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&uart3_rxbuff, 1); //接收, 存到recvData里
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  PID_Update pid_velocity;
  pid_velocity.Kp = 3000;
  pid_velocity.Ki = 1000;
  pid_velocity.Kd = 0;
  pid_velocity.integral = 300;
  pid_velocity.integral_limit =0.1;
  pid_velocity.integrator_separation_threshold = 0;
  pid_velocity.prev_error = 0;
  pid_velocity.prev_output = 0;
  pid_velocity.time_interval =0.01;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); // 打开红灯

	Encoder_Init(); //初始化编码器
	Control_Init(); //初始化控制算法
  	OLED_Init();  //OLED初始化IIC2
	Set_PWM(&htim1, 50, TIM_CHANNEL_1); //设置初始PWM
	Set_PWM(&htim2, 50, TIM_CHANNEL_2);
	
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); // 打开绿灯	  

	//HAL_UART_Transmit(&huart1, (uint8_t *)"test", 5,50); //串口发送字符串uart1
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&usart1Recv.recvData,1); //开启uart1接收 openmv
	HAL_UART_Receive_IT(&huart3,(uint8_t *)&uart3_rxbuff, 1);// 开启uart3接收 蓝牙
	
//	MPU6050_Init(Sensor_I2C1_Serch()); //初始化角度传感器

	uint8_t  SendBuffer[30]={"Hello,world\r\n"};
	OLED_ShowNum(1,1,1,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //menu1();
	  if(encoderTime >= 10){
		  encoderTime = 0;
		  float speedR = Get_Wheel_Speed_R();
		  float speedL = Get_Wheel_Speed_L();
		  int32_t getLL = getL();
		  char strR[20];
		  char strL[20];
		  sprintf(strR, "Right: %.3f", speedR);
		  sprintf(strL, "Left:  %.3f", speedL);
		  OLED_ShowString(1,1,strL);
		  OLED_ShowString(2,1,strR); //在OLED上显示转速
	  
		float actualspeedR_pwm = PID_Controller(&pid_velocity,0.5,speedR); //根据实际转速, 用PID得到占空比
		float actualspeedL_pwm = PID_Controller(&pid_velocity,0.5,speedL);
		Motor(actualspeedL_pwm,actualspeedR_pwm); //设置得到的占空比
		char str[50];
		sprintf(str, "speed:%f, %f, %f, %f\n", speedR, speedL, actualspeedR_pwm, actualspeedL_pwm); //传到串口2
		HAL_UART_Transmit(&huart2, (uint8_t *)str, sizeof(str),50);
	  }

	  	//BlueToothTransmitData();

	  //Control_Loop_Angle0(); //闭环控制

	  

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//	if(!usart1Recv.recvFlag){ //接收数据, 将接收到的数据再发送到串口, 同时显示在oled第2行
//        usart1Recv.recvFlag = 0;
//        HAL_UART_Transmit_IT(&huart1, usart1Recv.recvBuff, usart1Recv.recvNum);             // 发送数据
//        usart1Recv.recvNum = 0;
//        OLED_ShowStr(0,32,usart1Recv.recvBuff,2);
//        }
	  
//    if (reset_flag){
//      //Control
//    }
//    else{
//      System_Reset(); 
//    }

//	  if(mpu6050Time >= 500){
//		  mpu6050Time = 0;
//		  MPU6050_Read_Accel();
//		  MPU6050_Read_Gyro();
//		  MPU6050_Read_Temp();
//	  }
	  //HAL_UART_Transmit(&huart3, "Hello,world\r\n", 30, 500);
	  
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

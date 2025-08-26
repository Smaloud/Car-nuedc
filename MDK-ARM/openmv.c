/********************************* (C) COPYRIGHT **********************************
* File Name						    : openmv.c
* Author							: 混分巨兽龙某某
* Version							: V1.0.0
* Data								: 2023/11/03
* Contact							: QQ:1178305328
* Description					    : OpenMV and STM32 Communication Files
***********************************************************************************/
#include "openmv.h"
#include "usart.h"
#include "stdio.h"
//#include "OLED_Function.h"
//#include "OLED_IIC_Config.h"
#include <cstdlib>

uint8_t Angle = 0;
uint8_t Number2 = 0;
uint8_t Number3 = 0;
uint8_t Send1 = 0;
uint8_t Send2 = 0;
uint8_t Send3 = 0;

uint8_t sendBuf[4];

//static uint8_t Number4 = 0;

/* STM32接收端处理OpenMV传输的数据 */
void OpenMV_Data_Receive(int16_t OpenMV_Data)
{

	/* 计数变量 */
	static uint8_t RxCounter=0;			//计数变量
	/* 数据接收数组 */
	static uint16_t RxBuffer[6]={0};
	/* 数据传输状态位 */
	static uint8_t RxState = 0;
	
	/* 判断数据是否为有效数据，解码 */
	if(RxState == 0 && OpenMV_Data == 0xFE)				//0xFE帧头
	{
		RxState = 1;																//状态位改变
		RxBuffer[RxCounter++] = OpenMV_Data;				//将数据放入接收数组
	}
	else if(RxState == 1 && OpenMV_Data == 0xBC)	//0xBC帧头
	{
		RxState = 2;																//状态位改变
		RxBuffer[RxCounter++] = OpenMV_Data;				//将数据放入接收数组
	}
	else if(RxState == 2)													//读取目标数据（根据实际情况处理）
	{
		if(OpenMV_Data!=0xEF){
		RxBuffer[RxCounter++] = OpenMV_Data;	
			
			//将数据放入接收数组
			if(RxCounter==6){
					/* 计数和状态位归零 */
			RxCounter = 0;
			RxState = 0;
			/* 清空存放数据的数组 */
			for(int i = 0;i < 6; i++)
			{
				RxBuffer[i] = 0x00;
			}
			
			}						
			
		}	else {
			Number3 = RxBuffer[RxCounter-1];
			Number2 = RxBuffer[RxCounter-2];
			Angle = RxBuffer[RxCounter-3];
			
			int signedNumber1 = (int)Angle;
			int signedNumber2 = (int)Number2;
			int signedNumber3 = (int)Number2;

			if(signedNumber1 > 127){
				signedNumber1 -= 256;
			}
			
			if(signedNumber2 > 127){
				signedNumber2 -= 256;
			}
			
			if(signedNumber3 > 127){
				signedNumber3 -= 256;
			}
			
	  	char str[25];
		sprintf(str, "%d", signedNumber1);
		//OLED_ShowStr(0,0,(unsigned char *)str,2);

			
			RxCounter = 0;
			RxState = 0;
			
		}
	}
//		else if(RxState == 3)													//读取目标数据（根据实际情况处理）
//	{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//将数据放入接收数组
//			RxState = 4;															//状态位改变
//			Number2 = RxBuffer[RxCounter-1];
//			char str[25];
//			sprintf(str, "%d", Number2);
//			/* OLED显示目标数字 */
//		OLED_ShowStr(20,32,str,2);
//		
//	}
//		else if(RxState == 4)													//读取目标数据（根据实际情况处理）
//	{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//将数据放入接收数组
//			RxState = 5;															//状态位改变
//			Number3 = OpenMV_Data;
//			char str[25];
//			sprintf(str, "%d", Number3);
//			/* OLED显示目标数字 */
//		OLED_ShowStr(0,48,str,2);
//	}
//			else if(RxState == 5)													//读取目标数据（根据实际情况处理）
//	{
//		if(RxCounter>=6||OpenMV_Data == 0xEF)
//		{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//将数据放入接收数组
//			RxState = 6;															//状态位改变
//			Number4 = RxBuffer[RxCounter-1];
//			char str[25];
//			sprintf(str, "%d", Number4);
//			/* OLED显示目标数字 */
//		OLED_ShowStr(30,48,str,2);
//		}
//	}
	

	else			//整体的接收异常
	{
		/* 计数和状态位归零 */
		RxCounter = 0;
		RxState = 0;
		/* 清空存放数据的数组 */
		for(int i = 0;i < 6; i++)
		{
			RxBuffer[i] = 0x00;
		}
	}
}

void SendDataToOpenmv(void)
{
	uint8_t i;
	//加上发送给openmv 的数据的代码 (帧头， 模板匹配模式选择标志位，模式2所需要匹配的数字，帧尾)   //不需要很高的发送频率
		
		
		for(i = 0; i <= 4; i++)   //将TASK和TargetNum打包一次性发送给openmv
		{
			sprintf((char *)sendBuf, "*%d%d&", Send1, Send2);    
			HAL_UART_Transmit(&huart1, sendBuf, sizeof(sendBuf), 1000);
		}
}

/********************************* (C) COPYRIGHT **********************************
* File Name						    : openmv.c
* Author							: ��־�����ĳĳ
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

/* STM32���ն˴���OpenMV��������� */
void OpenMV_Data_Receive(int16_t OpenMV_Data)
{

	/* �������� */
	static uint8_t RxCounter=0;			//��������
	/* ���ݽ������� */
	static uint16_t RxBuffer[6]={0};
	/* ���ݴ���״̬λ */
	static uint8_t RxState = 0;
	
	/* �ж������Ƿ�Ϊ��Ч���ݣ����� */
	if(RxState == 0 && OpenMV_Data == 0xFE)				//0xFE֡ͷ
	{
		RxState = 1;																//״̬λ�ı�
		RxBuffer[RxCounter++] = OpenMV_Data;				//�����ݷ����������
	}
	else if(RxState == 1 && OpenMV_Data == 0xBC)	//0xBC֡ͷ
	{
		RxState = 2;																//״̬λ�ı�
		RxBuffer[RxCounter++] = OpenMV_Data;				//�����ݷ����������
	}
	else if(RxState == 2)													//��ȡĿ�����ݣ�����ʵ���������
	{
		if(OpenMV_Data!=0xEF){
		RxBuffer[RxCounter++] = OpenMV_Data;	
			
			//�����ݷ����������
			if(RxCounter==6){
					/* ������״̬λ���� */
			RxCounter = 0;
			RxState = 0;
			/* ��մ�����ݵ����� */
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
//		else if(RxState == 3)													//��ȡĿ�����ݣ�����ʵ���������
//	{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//�����ݷ����������
//			RxState = 4;															//״̬λ�ı�
//			Number2 = RxBuffer[RxCounter-1];
//			char str[25];
//			sprintf(str, "%d", Number2);
//			/* OLED��ʾĿ������ */
//		OLED_ShowStr(20,32,str,2);
//		
//	}
//		else if(RxState == 4)													//��ȡĿ�����ݣ�����ʵ���������
//	{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//�����ݷ����������
//			RxState = 5;															//״̬λ�ı�
//			Number3 = OpenMV_Data;
//			char str[25];
//			sprintf(str, "%d", Number3);
//			/* OLED��ʾĿ������ */
//		OLED_ShowStr(0,48,str,2);
//	}
//			else if(RxState == 5)													//��ȡĿ�����ݣ�����ʵ���������
//	{
//		if(RxCounter>=6||OpenMV_Data == 0xEF)
//		{
//		RxBuffer[RxCounter++] = OpenMV_Data;				//�����ݷ����������
//			RxState = 6;															//״̬λ�ı�
//			Number4 = RxBuffer[RxCounter-1];
//			char str[25];
//			sprintf(str, "%d", Number4);
//			/* OLED��ʾĿ������ */
//		OLED_ShowStr(30,48,str,2);
//		}
//	}
	

	else			//����Ľ����쳣
	{
		/* ������״̬λ���� */
		RxCounter = 0;
		RxState = 0;
		/* ��մ�����ݵ����� */
		for(int i = 0;i < 6; i++)
		{
			RxBuffer[i] = 0x00;
		}
	}
}

void SendDataToOpenmv(void)
{
	uint8_t i;
	//���Ϸ��͸�openmv �����ݵĴ��� (֡ͷ�� ģ��ƥ��ģʽѡ���־λ��ģʽ2����Ҫƥ������֣�֡β)   //����Ҫ�ܸߵķ���Ƶ��
		
		
		for(i = 0; i <= 4; i++)   //��TASK��TargetNum���һ���Է��͸�openmv
		{
			sprintf((char *)sendBuf, "*%d%d&", Send1, Send2);    
			HAL_UART_Transmit(&huart1, sendBuf, sizeof(sendBuf), 1000);
		}
}

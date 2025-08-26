#include "BlueTooth.h"
#include "openmv.h"
#include "usart.h"

uint8_t BlueToothSendBuf[5] = {0x52, 0x21, 0x00, 0x00, 0xf2};
uint8_t NumS1, NumS2;

uint16_t BlueRxBuffer[5]={0};
uint8_t NumB1, NumB2;

void BlueToothTransmitData(void){ //��������
   	uint8_t i;
	  BlueToothSendBuf[2] = 39;
	  BlueToothSendBuf[3] = 76;
	  //BlueToothSendBuf[4] = TASK;
	
		for(i = 0; i <= 5; i++)   //�����ַ�����λ������ͨ��HEX���������Ƿ���ȷ
		{
			HAL_UART_Transmit(&huart3, &BlueToothSendBuf[i], 1, 1000);
		}
}






void BlueTooth_Receive_Data(uint8_t com_data) //��������: ֡ͷ0x52, 0x21, ��, ��, NumB1, NumB2, ֡β0xf2
{
		uint8_t i;
		static uint8_t BlueRxCounter=0;//����
		static uint8_t BlueRxState = 0;	
		//static uint8_t BlueRxFlag = 0;    //�����ɹ����ձ�־λ
	 //�����ɹ����ձ�־λ��ʵ���Բ�Ҫ����OneTargertNum��OneLoadFlagֱ�Ӳ�����һ�ε�Ҳûɶ������һ�㲻����ֿ�����������ܿ��ˢ��������
	 //��û��Ҫ����߽�OneTargertNum��OneLoadFlagʹ��һ��֮�����BlueRxFlagһͬ����

		if(BlueRxState==0&&com_data==0x52)  //0x52֡ͷ
		{
			
			BlueRxState=1;
			BlueRxBuffer[BlueRxCounter++]=com_data;  
		}

		else if(BlueRxState==1&&com_data==0x21)  //0x21֡ͷ
		{
			BlueRxState=2;
			BlueRxBuffer[BlueRxCounter++]=com_data;
		}
		
		else if(BlueRxState==2)
		{
			 
			BlueRxBuffer[BlueRxCounter++]=com_data;
			if(BlueRxCounter>=5||com_data == 0xf2)    
			{
				BlueRxState=3;
				
				//BlueRxFlag1=1;    //�ɹ�����һ�����ݣ������������ж��Ƿ������������ 
				
//				OneTargertRoom =  BlueRxBuffer1[BlueRxCounter-5];     //[BlueRxCounter-5]��R  Ҳ��Ҳ��82��Ҳ��ʮ�����Ƶ�52    ��ȷ��֡ͷ1 
//				OneLoadFlag =  BlueRxBuffer1[BlueRxCounter-4];  //[BlueRxCounter-4]  ������ʮ���Ƶ�33��Ҳ��ʮ�����Ƶ�21      ��ȷ��֡ͷ2    
				
				NumB1 =  BlueRxBuffer[BlueRxCounter-3];        
				NumB2 =  BlueRxBuffer[BlueRxCounter-2];          
				
				
				//���������������RxCounter1-1�ŵ���֡β

			}
		}

		else if(BlueRxState==3)		//����Ƿ���ܵ�������־
		{
				//if(BlueRxBuffer[BlueRxState-1] == 0xf2)    //����д������������������BlueRxState1-1������BlueRxCounter1-1��������ȫ�����Զ�������
			
			  if(BlueRxBuffer[BlueRxCounter-1] == 0xf2)    
				{
							
							//BlueRxFlag = 0;
							BlueRxState = 0;
							BlueRxCounter = 0;
						
				}
				else   //���մ���
				{
							BlueRxState  = 0;
							BlueRxCounter =0;
							for(i=0;i<5;i++)
							{
									BlueRxBuffer[i]=0x00;      //�����������������
							}
				}
		} 

		else   //�����쳣
		{
				BlueRxState = 0;
				BlueRxCounter = 0;
				for(i=0;i<5;i++)
				{
						BlueRxBuffer[i]=0x00;      //�����������������
				}
		}
}

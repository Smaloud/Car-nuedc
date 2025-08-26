#include "BlueTooth.h"
#include "openmv.h"
#include "usart.h"

uint8_t BlueToothSendBuf[5] = {0x52, 0x21, 0x00, 0x00, 0xf2};
uint8_t NumS1, NumS2;

uint16_t BlueRxBuffer[5]={0};
uint8_t NumB1, NumB2;

void BlueToothTransmitData(void){ //蓝牙发送
   	uint8_t i;
	  BlueToothSendBuf[2] = 39;
	  BlueToothSendBuf[3] = 76;
	  //BlueToothSendBuf[4] = TASK;
	
		for(i = 0; i <= 5; i++)   //发送字符，上位机可以通过HEX检验数据是否正确
		{
			HAL_UART_Transmit(&huart3, &BlueToothSendBuf[i], 1, 1000);
		}
}






void BlueTooth_Receive_Data(uint8_t com_data) //蓝牙接收: 帧头0x52, 0x21, 空, 空, NumB1, NumB2, 帧尾0xf2
{
		uint8_t i;
		static uint8_t BlueRxCounter=0;//计数
		static uint8_t BlueRxState = 0;	
		//static uint8_t BlueRxFlag = 0;    //蓝牙成功接收标志位
	 //蓝牙成功接收标志位其实可以不要。我OneTargertNum，OneLoadFlag直接采用上一次的也没啥，反正一般不会出现卡死的情况，很快就刷新数据了
	 //更没必要在外边将OneTargertNum，OneLoadFlag使用一次之后就与BlueRxFlag一同清零

		if(BlueRxState==0&&com_data==0x52)  //0x52帧头
		{
			
			BlueRxState=1;
			BlueRxBuffer[BlueRxCounter++]=com_data;  
		}

		else if(BlueRxState==1&&com_data==0x21)  //0x21帧头
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
				
				//BlueRxFlag1=1;    //成功接收一次数据，可以用来做判断是否采用最新数据 
				
//				OneTargertRoom =  BlueRxBuffer1[BlueRxCounter-5];     //[BlueRxCounter-5]是R  也即也即82，也即十六进制的52    正确的帧头1 
//				OneLoadFlag =  BlueRxBuffer1[BlueRxCounter-4];  //[BlueRxCounter-4]  里面是十进制的33，也即十六进制的21      正确的帧头2    
				
				NumB1 =  BlueRxBuffer[BlueRxCounter-3];        
				NumB2 =  BlueRxBuffer[BlueRxCounter-2];          
				
				
				//如果接收正常，则RxCounter1-1放的是帧尾

			}
		}

		else if(BlueRxState==3)		//检测是否接受到结束标志
		{
				//if(BlueRxBuffer[BlueRxState-1] == 0xf2)    //这里写错啦！！！！！不是BlueRxState1-1，而是BlueRxCounter1-1！！！！全部都自动清零啦
			
			  if(BlueRxBuffer[BlueRxCounter-1] == 0xf2)    
				{
							
							//BlueRxFlag = 0;
							BlueRxState = 0;
							BlueRxCounter = 0;
						
				}
				else   //接收错误
				{
							BlueRxState  = 0;
							BlueRxCounter =0;
							for(i=0;i<5;i++)
							{
									BlueRxBuffer[i]=0x00;      //将存放数据数组清零
							}
				}
		} 

		else   //接收异常
		{
				BlueRxState = 0;
				BlueRxCounter = 0;
				for(i=0;i<5;i++)
				{
						BlueRxBuffer[i]=0x00;      //将存放数据数组清零
				}
		}
}

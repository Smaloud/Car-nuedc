#ifndef __OPENMV_H
#define __OPENMV_H

#include "stm32f1xx.h"

void OpenMV_Data_Receive(int16_t OpenMV_Data);		/* STM32接收端处理OpenMV传输的数据 */
void SendDataToOpenmv(void);

#endif

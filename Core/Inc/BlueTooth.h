#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "stm32f1xx.h"

extern uint8_t BlueToothSendBuf[5];
extern uint8_t NumS1, NumS2;

extern uint16_t BlueRxBuffer[5];
extern uint8_t NumB1, NumB2;

void BlueToothTransmitData(void);
void BlueTooth_Receive_Data(uint8_t com_data);

#endif

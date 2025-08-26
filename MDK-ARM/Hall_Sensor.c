#include "BlueTooth.h"
#include "openmv.h"
#include "usart.h"
#include "Encoder.h"
#include "stm32f1xx_hal.h"
#include "Hall_Sensor.h"

GPIO_PinState Read_Hall(void)
{
 GPIO_PinState read_GPIO;   
 read_GPIO = HAL_GPIO_ReadPin(HALL_SENSOR_GPIO_Port,HALL_SENSOR_Pin);
 return read_GPIO;
}

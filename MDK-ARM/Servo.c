#include "Servo.h"
#include "tim.h"

void Servo_SetAngle(int angle , int time){
	 int  PWM ;
	 PWM = 100 * angle/ 9 +500-1;
   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);
	 HAL_Delay(time);
 }

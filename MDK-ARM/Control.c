#include "Control.h"
#include "Encoder.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>
#include "usart.h" // ���봮��ͷ�ļ�
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
//#include "OLED_IIC_Config.h"
//#include "OLED_Function.h"
//#include "OLED_Front.h"
#include "openmv.h"
// ����һЩ��Ҫ�ĳ���
#define TIME_INTERVAL 0.1  // ʱ��������λΪ�루���磬ÿ0.1���ȡһ�α�����ֵ��
#define SETPOINT_ANGLE 0.0 // �趨��Ŀ��Ƕȣ���λΪ��
#define PWM_MAX  100
// �����ⲿ��PWM�������
extern TIM_HandleTypeDef htim2; // ����ʹ�õ���TIM2������PWM�ź�
extern TIM_HandleTypeDef htim1; // ����ʹ�õ���TIM1������PWM�ź�
volatile uint8_t reset_flag;  
extern uint8_t Angle,Number2, Number3;
float angle = 0.0f;

PID_Update pid_velocity;

void Control_Init(void)
{
	//OLED_ShowStr(0,16,"loop",2); 

    // ��ʼ��������
    //Encoder_Init();

    // ����PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);// ����ʹ��TIM3��ͨ��1����PWM�ź�
}

void Set_direction_L(uint8_t state){
	if(state==0) {
		//��ת
	A1_RESET;
	A2_SET;
	}else{
		//��ת
	A1_SET;
	A2_RESET;
	}
}

void Set_direction_R(uint8_t state){
	if(state==0) {
		//��ת
	B1_RESET;
	B2_SET;
	}else{
		//��ת
	B1_SET;
	B2_RESET;
	}
}

void Set_PWM(TIM_HandleTypeDef *htim, float duty_cycle, uint32_t channel)
{
	if(duty_cycle>100){duty_cycle=100;}
    // ����TIM��Period��1000
    uint32_t compare_value = (duty_cycle / 100.0f) * 1000;

    // ����PWMռ�ձ�
    __HAL_TIM_SET_COMPARE(htim, channel, compare_value);
}

void Set_Velocity(TIM_HandleTypeDef *htim,uint32_t channel, float speed, float max_speed){
 
 float duty_cycle = (speed / max_speed) * PWM_MAX;

    // ȷ��ռ�ձ�����Ч��Χ��
    if (duty_cycle > PWM_MAX) {
        duty_cycle = PWM_MAX;
    }
    if (duty_cycle < 0) {
        duty_cycle = 0;
    }
    Set_PWM(htim, channel, duty_cycle);
    }


void Motor(uint16_t L, uint16_t R){
	if(L>0) Set_direction_L(0);
	else Set_direction_L(1);
	if(R>0) Set_direction_R(0);
	else Set_direction_R(1);
	Set_PWM(&htim1, L, TIM_CHANNEL_1);
	Set_PWM(&htim2, R, TIM_CHANNEL_2);
	
}
//�нǶ����к���
void Control_Loop_AngleX(void)
{
	

    // ����OpenMV�����ƫ��Ƕ�
//    if (HAL_UART_Receive(&huart1, (uint8_t*)&angle, sizeof(angle), HAL_MAX_DELAY) == HAL_OK)
// 
        // ����PID������
        float pid_output = PID_Controller(&pid_velocity, angle, SETPOINT_ANGLE);

        // ��PID������ת��Ϊ�����ֵ�PWMռ�ձ�
        float left_wheel_pwm = 50.0f - pid_output;  // ����50%ռ�ձȵ���
        float right_wheel_pwm = 50.0f + pid_output; // ����50%ռ�ձȵ���

        // ����PWMռ�ձ���0%��100%֮��
        if (left_wheel_pwm > 100.0f) left_wheel_pwm = 100.0f;
        if (left_wheel_pwm < 0.0f) left_wheel_pwm = 0.0f;
        if (right_wheel_pwm > 100.0f) right_wheel_pwm = 100.0f;
        if (right_wheel_pwm < 0.0f) right_wheel_pwm = 0.0f;

        // ���������ֵ�PWMռ�ձ�
        Set_PWM(&htim1, left_wheel_pwm, TIM_CHANNEL_1);
        Set_PWM(&htim2, right_wheel_pwm, TIM_CHANNEL_2);
        
}

//�޽Ƕ����к���
void Control_Loop_Angle0(void){
//���������ٶ�Ϊ��׼��cm/s��
float left_wheel_pwm = 50.0f;
Set_PWM(&htim1, left_wheel_pwm, TIM_CHANNEL_1);
float Standard_Speed_Cm = Get_Wheel_Speed_L() * 100;
float SetPoint_Speed =  Standard_Speed_Cm;
float Actual_Speed = Get_Wheel_Speed_R() * 100; 
float pid_output = PID_Controller(&pid_velocity, SetPoint_Speed, Actual_Speed);
Set_Velocity(&htim2, TIM_CHANNEL_2, Actual_Speed + pid_output, 10);
}

//�ۺ����к���
void Control_Loop(void){
    if (angle > 5 || angle < -5){
        Control_Loop_AngleX();
    }
    else{
        Control_Loop_Angle0();
    }
}
//}

// #include "main.h"
// #include "Control.h"
// #include "usart.h" // ���봮��ͷ�ļ�

// int main(void)
// {
//     // ��ʼ��HAL��
//     HAL_Init();
    
//     // ����ϵͳʱ��
//     SystemClock_Config();
    
//     // ��ʼ���������úõ�����
//     MX_GPIO_Init();
//     MX_TIM2_Init(); // ����TIM2�ĳ�ʼ��������ΪMX_TIM2_Init
//     MX_TIM3_Init(); // ����TIM3�ĳ�ʼ��������ΪMX_TIM3_Init
//     MX_TIM4_Init(); // ����TIM4�ĳ�ʼ��������ΪMX_TIM4_Init
//     MX_USART2_UART_Init(); // ����USART2�ĳ�ʼ��������ΪMX_USART2_UART_Init

//     // ��ʼ������ģ��
//     Control_Init();

//     while (1)
//     {
//         // ִ�п���ѭ��
//         Control_Loop();
//     }
// }

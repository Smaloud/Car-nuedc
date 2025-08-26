#include "Control.h"
#include "Encoder.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>
#include "usart.h" // 引入串口头文件
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
//#include "OLED_IIC_Config.h"
//#include "OLED_Function.h"
//#include "OLED_Front.h"
#include "openmv.h"
// 定义一些必要的常数
#define TIME_INTERVAL 0.1  // 时间间隔，单位为秒（例如，每0.1秒读取一次编码器值）
#define SETPOINT_ANGLE 0.0 // 设定的目标角度，单位为度
#define PWM_MAX  100
// 声明外部的PWM句柄变量
extern TIM_HandleTypeDef htim2; // 假设使用的是TIM2来产生PWM信号
extern TIM_HandleTypeDef htim1; // 假设使用的是TIM1来产生PWM信号
volatile uint8_t reset_flag;  
extern uint8_t Angle,Number2, Number3;
float angle = 0.0f;

PID_Update pid_velocity;

void Control_Init(void)
{
	//OLED_ShowStr(0,16,"loop",2); 

    // 初始化编码器
    //Encoder_Init();

    // 启动PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);// 假设使用TIM3的通道1产生PWM信号
}

void Set_direction_L(uint8_t state){
	if(state==0) {
		//正转
	A1_RESET;
	A2_SET;
	}else{
		//反转
	A1_SET;
	A2_RESET;
	}
}

void Set_direction_R(uint8_t state){
	if(state==0) {
		//正转
	B1_RESET;
	B2_SET;
	}else{
		//反转
	B1_SET;
	B2_RESET;
	}
}

void Set_PWM(TIM_HandleTypeDef *htim, float duty_cycle, uint32_t channel)
{
	if(duty_cycle>100){duty_cycle=100;}
    // 假设TIM的Period是1000
    uint32_t compare_value = (duty_cycle / 100.0f) * 1000;

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(htim, channel, compare_value);
}

void Set_Velocity(TIM_HandleTypeDef *htim,uint32_t channel, float speed, float max_speed){
 
 float duty_cycle = (speed / max_speed) * PWM_MAX;

    // 确保占空比在有效范围内
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
//有角度运行函数
void Control_Loop_AngleX(void)
{
	

    // 接收OpenMV传入的偏离角度
//    if (HAL_UART_Receive(&huart1, (uint8_t*)&angle, sizeof(angle), HAL_MAX_DELAY) == HAL_OK)
// 
        // 计算PID控制量
        float pid_output = PID_Controller(&pid_velocity, angle, SETPOINT_ANGLE);

        // 将PID控制量转换为左右轮的PWM占空比
        float left_wheel_pwm = 50.0f - pid_output;  // 基于50%占空比调整
        float right_wheel_pwm = 50.0f + pid_output; // 基于50%占空比调整

        // 限制PWM占空比在0%到100%之间
        if (left_wheel_pwm > 100.0f) left_wheel_pwm = 100.0f;
        if (left_wheel_pwm < 0.0f) left_wheel_pwm = 0.0f;
        if (right_wheel_pwm > 100.0f) right_wheel_pwm = 100.0f;
        if (right_wheel_pwm < 0.0f) right_wheel_pwm = 0.0f;

        // 设置左右轮的PWM占空比
        Set_PWM(&htim1, left_wheel_pwm, TIM_CHANNEL_1);
        Set_PWM(&htim2, right_wheel_pwm, TIM_CHANNEL_2);
        
}

//无角度运行函数
void Control_Loop_Angle0(void){
//调节左轮速度为基准（cm/s）
float left_wheel_pwm = 50.0f;
Set_PWM(&htim1, left_wheel_pwm, TIM_CHANNEL_1);
float Standard_Speed_Cm = Get_Wheel_Speed_L() * 100;
float SetPoint_Speed =  Standard_Speed_Cm;
float Actual_Speed = Get_Wheel_Speed_R() * 100; 
float pid_output = PID_Controller(&pid_velocity, SetPoint_Speed, Actual_Speed);
Set_Velocity(&htim2, TIM_CHANNEL_2, Actual_Speed + pid_output, 10);
}

//综合运行函数
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
// #include "usart.h" // 引入串口头文件

// int main(void)
// {
//     // 初始化HAL库
//     HAL_Init();
    
//     // 配置系统时钟
//     SystemClock_Config();
    
//     // 初始化所有配置好的外设
//     MX_GPIO_Init();
//     MX_TIM2_Init(); // 假设TIM2的初始化函数名为MX_TIM2_Init
//     MX_TIM3_Init(); // 假设TIM3的初始化函数名为MX_TIM3_Init
//     MX_TIM4_Init(); // 假设TIM4的初始化函数名为MX_TIM4_Init
//     MX_USART2_UART_Init(); // 假设USART2的初始化函数名为MX_USART2_UART_Init

//     // 初始化控制模块
//     Control_Init();

//     while (1)
//     {
//         // 执行控制循环
//         Control_Loop();
//     }
// }

#include "BlueTooth.h"
#include "openmv.h"
#include "usart.h"
#include "Encoder.h"
#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;

#define PULSES_PER_REVOLUTION       364  // ������ÿת������������Ҫ����ʵ�ʱ�����������д��
#define WHEEL_DIAMETER    0.062  // ���ӵ�ֱ������λΪ�ף���Ҫ����ʵ������ֱ����д��
#define PI                3.1415926
#define TIME_INTERVAL     0.01

static int32_t encoder_count_prev_L = 0;
static int32_t encoder_count_prev_R = 0;
static int32_t encoder_count_current_L = 0;
static int32_t encoder_count_current_R = 0;
static float wheel_speed_R = 0.0;
static float wheel_speed_L = 0.0;
static float wheel_velocity_R = 0.0;
static float wheel_velocity_L = 0.0;
static int wheel_direction_R = 0;
static int wheel_direction_L = 0;


void Encoder_Init(void)
{
    // ʹ�� TIM3 �� GPIOA ʱ��
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 30000);
	__HAL_TIM_SET_COUNTER(&htim4, 30000);
}
//R-tim2 L-tim3
float Get_Wheel_Speed_R(void)
{
    // ��ȡ��ǰ����������ֵ
    encoder_count_current_R = __HAL_TIM_GET_COUNTER(&htim3)-30000;
	
    
//    // �������ֵ�Ĳ���
//    int32_t count_diff1 = encoder_count_current_R - encoder_count_prev_R;

//    // ������һ������ֵ
//    encoder_count_prev_R = encoder_count_current_R;

    // ת�ټ��㹫ʽ�����������ÿתһȦ����500�����壬ת�ٹ�ʽΪ��
    // Speed (RPM) = (Count Difference / Pulses per Revolution) * (60 / Time Interval)/ABEncoder
    wheel_speed_R = (float)encoder_count_current_R * (60.0/TIME_INTERVAL/PULSES_PER_REVOLUTION/4);
	
	wheel_velocity_R = - wheel_speed_R * PI * WHEEL_DIAMETER / 60;
	__HAL_TIM_SET_COUNTER(&htim3, 30000);
	
    return wheel_velocity_R;
}

float Get_Wheel_Speed_L(void)
{
    // ��ȡ��ǰ����������ֵ
    encoder_count_current_L = __HAL_TIM_GET_COUNTER(&htim4)-30000;
    
//    // �������ֵ�Ĳ���
//    int32_t count_diff2 = encoder_count_current_L - encoder_count_prev_L;

//    // ������һ������ֵ
//    encoder_count_prev_L = encoder_count_current_L;

    // ת�ټ��㹫ʽ�����������ÿתһȦ����500�����壬ת�ٹ�ʽΪ��
    // Speed (RPM) = (Count Difference / Pulses per Revolution) * (60 / Time Interval)/ABEncoder
    wheel_speed_L = (float)encoder_count_current_L * (60.0/TIME_INTERVAL/PULSES_PER_REVOLUTION/4);

	wheel_velocity_L = - wheel_speed_L * PI * WHEEL_DIAMETER / 60;
  __HAL_TIM_SET_COUNTER(&htim4, 30000);
    return wheel_velocity_L;
}

int32_t getL(void){
	return encoder_count_current_R;
}


// ��ȡ���ӷ���+1 �� -1��
int Get_Wheel_Direction_R(void)
{
    // ��ȡ��ǰ����������ֵ
    encoder_count_current_R = __HAL_TIM_GET_COUNTER(&htim3);

    // �������ֵ�Ĳ���
    int32_t count_diff = encoder_count_current_R - encoder_count_prev_R;

    // ������һ������ֵ
    encoder_count_prev_R = encoder_count_current_R;

    // ���ݼ���ֵ�ı仯ȷ������
    if (count_diff > 0)
    {
        wheel_direction_R = 1;  // ˳ʱ�뷽��
    }
    else if (count_diff < 0)
    {
        wheel_direction_R = -1; // ��ʱ�뷽��
    }
    else
    {
        wheel_direction_R = 0;  // û��ת��
    }

    return wheel_direction_R;
}

int Get_Wheel_Direction_L(void)
{
    // ��ȡ��ǰ����������ֵ
    encoder_count_current_L = __HAL_TIM_GET_COUNTER(&htim4);

    // �������ֵ�Ĳ���
    int32_t count_diff = encoder_count_current_L - encoder_count_prev_L;

    // ������һ������ֵ
    encoder_count_prev_L = encoder_count_current_L;

    // ���ݼ���ֵ�ı仯ȷ������
    if (count_diff > 0)
    {
        wheel_direction_L = 1;  // ˳ʱ�뷽��
    }
    else if (count_diff < 0)
    {
        wheel_direction_L = -1; // ��ʱ�뷽��
    }
    else
    {
        wheel_direction_L = 0;  // û��ת��
    }

    return wheel_direction_L;
}


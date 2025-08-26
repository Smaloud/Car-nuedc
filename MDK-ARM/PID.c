#include "PID.h"
#include "math.h"
// PID����
// float Kp = 2000.0f;
// float Ki = 3000.0f;
// float Kd = 50.0f;
// float integrator_min = -500.0f;  // �����޷���Сֵ
// float integrator_max = 500.0f;   // �����޷����ֵ
// float integrator_separation_threshold = 0.1f; // ���ַ�����ֵ

// // PID״̬����
// float prev_error = 0.0f;
// float integral = 0.0f;

    // ���ַ���
    // if (fabs(error) < integrator_separation_threshold)
    // {
    //     integral += error * dt;
    // }
    
float PID_Controller(PID_Update *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;

    if (fabs(error) <pid->integrator_separation_threshold)//���ַ���
     {
          pid->integral += error * pid->time_interval;
     }

    if (pid->integral > pid->integral_limit) {//�����޷�
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    
    float derivative = (error - pid->prev_error) / pid->time_interval;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // ���浱ǰ���ֵ
    pid->prev_output = output;
    return output;
}

float PID_Controller_increment(PID_Update *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    float integral_increment;
    float derivative;
    float output;

    // ��������ַ��룩
    if (fabs(error) < pid->integrator_separation_threshold) {
        integral_increment = error * pid->time_interval;
    } else {
        integral_increment = 0;
    }
    
    pid->integral += integral_increment;
    
    // �����޷�
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    // ΢����
    derivative = (error - pid->prev_error) / pid->time_interval;
    
    // ������ PID ���
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // ��������
    float delta_output = output - pid->prev_output;

    // ���浱ǰ״̬
    pid->prev_output = output;
    pid->prev_error = error;
    
    return delta_output;
}

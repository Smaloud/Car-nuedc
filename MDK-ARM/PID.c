#include "PID.h"
#include "math.h"
// PID参数
// float Kp = 2000.0f;
// float Ki = 3000.0f;
// float Kd = 50.0f;
// float integrator_min = -500.0f;  // 积分限幅最小值
// float integrator_max = 500.0f;   // 积分限幅最大值
// float integrator_separation_threshold = 0.1f; // 积分分离阈值

// // PID状态变量
// float prev_error = 0.0f;
// float integral = 0.0f;

    // 积分分离
    // if (fabs(error) < integrator_separation_threshold)
    // {
    //     integral += error * dt;
    // }
    
float PID_Controller(PID_Update *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;

    if (fabs(error) <pid->integrator_separation_threshold)//积分分离
     {
          pid->integral += error * pid->time_interval;
     }

    if (pid->integral > pid->integral_limit) {//积分限幅
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    
    float derivative = (error - pid->prev_error) / pid->time_interval;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 保存当前输出值
    pid->prev_output = output;
    return output;
}

float PID_Controller_increment(PID_Update *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    float integral_increment;
    float derivative;
    float output;

    // 积分项（积分分离）
    if (fabs(error) < pid->integrator_separation_threshold) {
        integral_increment = error * pid->time_interval;
    } else {
        integral_increment = 0;
    }
    
    pid->integral += integral_increment;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }

    // 微分项
    derivative = (error - pid->prev_error) / pid->time_interval;
    
    // 增量型 PID 输出
    output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 计算增量
    float delta_output = output - pid->prev_output;

    // 保存当前状态
    pid->prev_output = output;
    pid->prev_error = error;
    
    return delta_output;
}

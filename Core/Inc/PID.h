#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral;
    float prev_output;
    float integrator_separation_threshold;
    float integral_limit; // 保存上一次输出值
    float time_interval;
} PID_Update;

float PID_Controller(PID_Update *pid, float setpoint, float measured_value);
float PID_Controller_increment(PID_Update *pid, float setpoint, float measured_value);

#endif // PID_H

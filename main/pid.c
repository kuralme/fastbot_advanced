#include "pid.h"

void pid_init(PID_t *pid, float kp, float ki, float kd, float kff)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->out_min = -255;
    pid->out_max = 255;
    pid_reset(pid);
}

void pid_reset(PID_t *pid)
{
    pid->integral = 0.0;
    pid->prev_error = 0.0;
    pid->setpoint = 0.0;
}

int pid_compute(PID_t *pid, float measured, float dt)
{
    float error = pid->setpoint - measured;

    float feed_forward = pid->setpoint * pid->kff;
    pid->integral += error * dt;
    float i_limit = 50.0;
    if (pid->integral > i_limit)
        pid->integral = i_limit;
    if (pid->integral < -i_limit)
        pid->integral = -i_limit;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Combine terms FF + PID
    float output = feed_forward + (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Output clamping
    if (output > pid->out_max)
        output = pid->out_max;
    if (output < pid->out_min)
        output = pid->out_min;
    return (int)output;
}
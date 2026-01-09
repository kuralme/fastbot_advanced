#include "pid.h"

void pid_init(PID_t *pid, float kp, float ki, float kd, int min, int max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = min;
    pid->out_max = max;
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
    pid->integral += error * dt;

    // Anti-windup clamping
    if (pid->integral > pid->out_max)
        pid->integral = pid->out_max;
    else if (pid->integral < pid->out_min)
        pid->integral = pid->out_min;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    if (output > pid->out_max)
        return pid->out_max;
    if (output < pid->out_min)
        return pid->out_min;
    return (int)output;
}
#include "pid.h"
#include "common.h"

void pid_init(PID_t *pid, PID_t config)
{
    pid->kp = config.kp;
    pid->ki = config.ki;
    pid->kd = config.kd;
    pid->kff = config.kff;
    pid->out_min = PID_OUT_MIN;
    pid->out_max = PID_OUT_MAX;
    pid_reset(pid);
}

void pid_reset(PID_t *pid)
{
    pid->integral = 0.0F;
    pid->prev_error = 0.0F;
    pid->setpoint = 0.0F;
}

int pid_compute(PID_t *pid, float measured)
{
    float error = pid->setpoint - measured;

    float feed_forward = pid->setpoint * pid->kff;

    pid->integral += error * CNT_TS;
    if (pid->integral > PID_INT_LIMIT)
    {
        pid->integral = PID_INT_LIMIT;
    }
    if (pid->integral < -PID_INT_LIMIT)
    {
        pid->integral = -PID_INT_LIMIT;
    }
    float derivative = (error - pid->prev_error) / CNT_TS;
    pid->prev_error = error;

    // Combine terms FF + PID
    float output = feed_forward + (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    // Output clamping
    if (output > pid->out_max)
    {
        output = pid->out_max;
    }
    if (output < pid->out_min)
    {
        output = pid->out_min;
    }
    return (int)output;
}
#ifndef PID_H
#define PID_H

typedef struct
{
    float kp, ki, kd;
    float setpoint;
    float integral;
    float prev_error;
    int out_min, out_max;
} PID_t;

void pid_init(PID_t *pid, float kp, float ki, float kd, int min, int max);
int pid_compute(PID_t *pid, float measured, float dt);
void pid_reset(PID_t *pid);

#endif // PID_H
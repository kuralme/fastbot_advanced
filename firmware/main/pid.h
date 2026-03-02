#ifndef PID_H
#define PID_H

#define PID_INT_LIMIT 50.0F
#define PID_OUT_MAX 255.0F
#define PID_OUT_MIN -255.0F

// Tuning parameters
#define PID_KP_DEFAULT 150.0F
#define PID_KI_DEFAULT 50.0F
#define PID_KD_DEFAULT 10.0F
#define PID_KFF_DEFAULT 18.0F

typedef struct
{
    float kp, ki, kd, kff;
    float setpoint;
    float integral;
    float prev_error;
    float out_min, out_max;
} PID_t;

void pid_init(PID_t *pid, PID_t config);
int pid_compute(PID_t *pid, float measured);
void pid_reset(PID_t *pid);

#endif // PID_H
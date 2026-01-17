#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "driver/ledc.h"

void configure_motors(void);
void set_motor_speeds(int left_pwm, int right_pwm);
int apply_deadzone(int pwm);

#endif // MOTOR_DRIVER_H
#include "motor_driver.h"

#define LEFT_MOTOR_FORWARD_GPIO 25
#define LEFT_MOTOR_BACKWARD_GPIO 26
#define RIGHT_MOTOR_FORWARD_GPIO 27
#define RIGHT_MOTOR_BACKWARD_GPIO 14

#define L_F_CH LEDC_CHANNEL_0
#define L_B_CH LEDC_CHANNEL_1
#define R_F_CH LEDC_CHANNEL_2
#define R_B_CH LEDC_CHANNEL_3

#define TIM_FREQ_HZ 5000
#define MOTOR_DEADZONE 40

void configure_motors(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = TIM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    int pins[] = {LEFT_MOTOR_FORWARD_GPIO, LEFT_MOTOR_BACKWARD_GPIO, RIGHT_MOTOR_FORWARD_GPIO, RIGHT_MOTOR_BACKWARD_GPIO};
    int channels[] = {L_F_CH, L_B_CH, R_F_CH, R_B_CH};

    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channels[i],
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = pins[i],
            .duty = 0,
            .hpoint = 0};
        ledc_channel_config(&ledc_channel);
    }
}

void set_motor_speeds(int left_pwm, int right_pwm)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, L_F_CH, (left_pwm > 0) ? left_pwm : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, L_B_CH, (left_pwm < 0) ? -left_pwm : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, R_F_CH, (right_pwm > 0) ? right_pwm : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, R_B_CH, (right_pwm < 0) ? -right_pwm : 0);

    for (int i = 0; i < 4; i++)
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
}

int apply_deadzone(int pwm)
{
    if (pwm == 0 || (pwm > -MOTOR_DEADZONE && pwm < MOTOR_DEADZONE))
        return 0;
    return pwm;
}
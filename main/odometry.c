#include <math.h>

#include "esp_attr.h"
#include "driver/gpio.h"

#include "odometry.h"

#define WHEEL_RADIUS 0.0325 // 65mm diameter wheels [meters]
#define WHEEL_BASE 0.125    // Distance between wheels [meters]
#define TICKS_PER_REV 1270  // Encoder ticks per wheel revolution

#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33
#define LEFT_ENC_A 34
#define LEFT_ENC_B 35

static volatile long left_tick_count = 0;
static volatile long right_tick_count = 0;
static double robot_x = 0, robot_y = 0, robot_theta = 0;

void IRAM_ATTR left_enc_cb()
{
    (gpio_get_level(LEFT_ENC_A) == gpio_get_level(LEFT_ENC_B)) ? left_tick_count++ : left_tick_count--;
}
void IRAM_ATTR right_enc_cb()
{
    (gpio_get_level(RIGHT_ENC_A) != gpio_get_level(RIGHT_ENC_B)) ? right_tick_count++ : right_tick_count--;
}

void configure_encoders()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LEFT_ENC_A) | (1ULL << LEFT_ENC_B) | (1ULL << RIGHT_ENC_A) | (1ULL << RIGHT_ENC_B),
        .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_ENC_A, left_enc_cb, NULL);
    gpio_isr_handler_add(RIGHT_ENC_A, right_enc_cb, NULL);
}

void update_odometry(nav_msgs__msg__Odometry *msg)
{
    static long last_left = 0, last_right = 0;
    long curr_l = left_tick_count, curr_r = right_tick_count;

    double d_left = (double)(curr_l - last_left) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);
    double d_right = (double)(curr_r - last_right) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);
    last_left = curr_l;
    last_right = curr_r;

    double d_dist = (d_right + d_left) / 2.0;
    double d_theta = (d_right - d_left) / WHEEL_BASE;

    robot_x += d_dist * cos(robot_theta + d_theta / 2.0);
    robot_y += d_dist * sin(robot_theta + d_theta / 2.0);
    robot_theta += d_theta;

    msg->pose.pose.position.x = robot_x;
    msg->pose.pose.position.y = robot_y;
    msg->pose.pose.orientation.z = sin(robot_theta / 2.0);
    msg->pose.pose.orientation.w = cos(robot_theta / 2.0);
}
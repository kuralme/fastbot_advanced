#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "odometry.h"
#include "pid.h"

#define WHEEL_RADIUS 0.0325F // 65mm diameter wheels [m]
#define WHEEL_BASE 0.125F    // Wheel separation [m]
#define TICKS_PER_REV 1265   // 4X CPR encoders

#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33
#define LEFT_ENC_A 34
#define LEFT_ENC_B 35
#define HALF_DIVISOR 2.0F

static robot_state_t shared_state;
static portMUX_TYPE tick_isr_spinlock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile long left_tick_count = 0;
static volatile long right_tick_count = 0;

void IRAM_ATTR left_enc_cb()
{
    portENTER_CRITICAL_ISR(&tick_isr_spinlock);
    (gpio_get_level(LEFT_ENC_A) == gpio_get_level(LEFT_ENC_B)) ? left_tick_count-- : left_tick_count++;
    portEXIT_CRITICAL_ISR(&tick_isr_spinlock);
}
void IRAM_ATTR right_enc_cb()
{
    portENTER_CRITICAL_ISR(&tick_isr_spinlock);
    (gpio_get_level(RIGHT_ENC_A) != gpio_get_level(RIGHT_ENC_B)) ? right_tick_count-- : right_tick_count++;
    portEXIT_CRITICAL_ISR(&tick_isr_spinlock);
}

void configure_encoders()
{
    // Configure all encoder GPIOs as inputs without forcing pull-ups here because
    // pins GPIO34/35 are input-only and do NOT have internal pull-up resistors.
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LEFT_ENC_A) | (1ULL << LEFT_ENC_B) | (1ULL << RIGHT_ENC_A) | (1ULL << RIGHT_ENC_B),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};
    gpio_config(&io_conf);

    // Enable internal pull-ups only for pins that support them (GPIO0-33)
    // RIGHT_ENC_A/B (GPIO32/33) may use internal pull-ups
    // LEFT_ENC_A/B (GPIO34/35) are input-only pads and require external pull-ups if needed
    gpio_set_pull_mode(RIGHT_ENC_A, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(RIGHT_ENC_B, GPIO_PULLUP_ONLY);

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE("ODOM", "gpio_install_isr_service failed: %d", isr_err);
    }

    esp_err_t isr_enc = gpio_isr_handler_add(LEFT_ENC_A, left_enc_cb, NULL);
    if (isr_enc != ESP_OK)
    {
        ESP_LOGE("ODOM", "Failed to add ISR for LEFT_ENC_A: %d", isr_enc);
    }
    isr_enc = gpio_isr_handler_add(RIGHT_ENC_A, right_enc_cb, NULL);
    if (isr_enc != ESP_OK)
    {
        ESP_LOGE("ODOM", "Failed to add ISR for RIGHT_ENC_A: %d", isr_enc);
    }
}

void get_robot_state(robot_state_t *copy)
{
    portENTER_CRITICAL(&state_spinlock);
    *copy = shared_state; // Thread-safe structural copy
    portEXIT_CRITICAL(&state_spinlock);
}

void update_robot_state()
{
    static long last_l = 0;
    static long last_r = 0;

    portENTER_CRITICAL(&tick_isr_spinlock);
    long curr_l = left_tick_count;
    long curr_r = right_tick_count;
    portEXIT_CRITICAL(&tick_isr_spinlock);

    const float meters_per_tick = (2.0F * (float)M_PI * WHEEL_RADIUS) / (float)TICKS_PER_REV;
    float d_l = (float)(curr_l - last_l) * meters_per_tick;
    float d_r = (float)(curr_r - last_r) * meters_per_tick;

    float d_dist = (d_r + d_l) / HALF_DIVISOR;
    float d_theta = (d_r - d_l) / WHEEL_BASE;

    float local_vel_l = d_l / PID_TS;
    float local_vel_r = d_r / PID_TS;

    // Update State
    portENTER_CRITICAL(&state_spinlock);

    // Heading used for movement calculations is the average of the start and end headings
    // during the interval, which is more accurate for small rotations
    float move_angle = (float)shared_state.theta + (d_theta / HALF_DIVISOR);

    shared_state.x += d_dist * cosf(move_angle);
    shared_state.y += d_dist * sinf(move_angle);
    shared_state.theta += d_theta;
    shared_state.vel_l = local_vel_l;
    shared_state.vel_r = local_vel_r;

    portEXIT_CRITICAL(&state_spinlock);
}
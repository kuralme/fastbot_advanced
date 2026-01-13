#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "odometry.h"

#define WHEEL_RADIUS 0.0325 // 65mm diameter wheels [m]
#define WHEEL_BASE 0.125    // Wheel separation [m]
#define TICKS_PER_REV 1270  // 4X CPR encoders

#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33
#define LEFT_ENC_A 34
#define LEFT_ENC_B 35

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
        .intr_type = GPIO_INTR_POSEDGE,
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

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE("ODOM", "gpio_install_isr_service failed: %d", err);
    }

    esp_err_t r;
    r = gpio_isr_handler_add(LEFT_ENC_A, left_enc_cb, NULL);
    if (r != ESP_OK)
        ESP_LOGE("ODOM", "Failed to add ISR for LEFT_ENC_A: %d", r);
    r = gpio_isr_handler_add(RIGHT_ENC_A, right_enc_cb, NULL);
    if (r != ESP_OK)
        ESP_LOGE("ODOM", "Failed to add ISR for RIGHT_ENC_A: %d", r);
}

void get_robot_state(robot_state_t *copy)
{
    portENTER_CRITICAL(&state_spinlock);
    *copy = shared_state; // Thread-safe structural copy
    portEXIT_CRITICAL(&state_spinlock);
}

void update_robot_state(float dt)
{
    static long last_l = 0, last_r = 0;
    long curr_l, curr_r;

    // Get ticks safely
    portENTER_CRITICAL(&tick_isr_spinlock);
    curr_l = left_tick_count;
    curr_r = right_tick_count;
    portEXIT_CRITICAL(&tick_isr_spinlock);

    // Distance moved since last call [m]
    double d_l = (double)(curr_l - last_l) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);
    double d_r = (double)(curr_r - last_r) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);
    last_l = curr_l;
    last_r = curr_r;

    // Update robot state safely
    portENTER_CRITICAL(&state_spinlock);

    double current_theta = shared_state.theta;
    double d_dist = (d_r + d_l) / 2.0;
    double d_theta = (d_r - d_l) / WHEEL_BASE;
    shared_state.vel_l = (float)(d_l / dt);
    shared_state.vel_r = (float)(d_r / dt);
    shared_state.x += d_dist * cos(current_theta + d_theta / 2.0);
    shared_state.y += d_dist * sin(current_theta + d_theta / 2.0);
    shared_state.theta += d_theta;

    portEXIT_CRITICAL(&state_spinlock);
}
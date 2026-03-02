#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "odometry.h"
#include "common.h"

enum
{
    TICKS_PER_REV = 1265, // 4X CPR encoders
    RIGHT_ENC_A = 32,
    RIGHT_ENC_B = 33,
    LEFT_ENC_A = 34,
    LEFT_ENC_B = 35
};

typedef struct
{
    robot_state_t state;       // Shared robot state (pose + velocity)
    portMUX_TYPE tick_mux;     // For raw encoder ticks
    portMUX_TYPE state_mux;    // For the calculated pose (x, y, theta)
    volatile long left_ticks;  // Raw encoder ticks since boot
    volatile long right_ticks; // Raw encoder ticks since boot
} odom_manager_t;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static odom_manager_t g_odom = {
    .state = {0},
    .tick_mux = portMUX_INITIALIZER_UNLOCKED,
    .state_mux = portMUX_INITIALIZER_UNLOCKED,
    .left_ticks = 0,
    .right_ticks = 0};

void IRAM_ATTR left_enc_cb()
{
    portENTER_CRITICAL_ISR(&g_odom.tick_mux);
    (gpio_get_level(LEFT_ENC_A) == gpio_get_level(LEFT_ENC_B)) ? g_odom.left_ticks-- : g_odom.left_ticks++;
    portEXIT_CRITICAL_ISR(&g_odom.tick_mux);
}
void IRAM_ATTR right_enc_cb()
{
    portENTER_CRITICAL_ISR(&g_odom.tick_mux);
    (gpio_get_level(RIGHT_ENC_A) != gpio_get_level(RIGHT_ENC_B)) ? g_odom.right_ticks-- : g_odom.right_ticks++;
    portEXIT_CRITICAL_ISR(&g_odom.tick_mux);
}
static void log_isr_error(esp_err_t err)
{
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE)
    {
        return;
    }
    ESP_LOGE("ODOM", "ISR service install failed: %d", err);
}
static void add_handler(gpio_num_t pin, gpio_isr_t handler)
{
    esp_err_t err = gpio_isr_handler_add(pin, handler, NULL);
    if (err == ESP_OK)
    {
        return;
    }
    ESP_LOGE("ODOM", "ISR add failed for pin %d", pin);
}
void configure_encoders()
{
    // Configure all encoder GPIOs as inputs without forcing pull-ups here because
    // pins GPIO34/35 are input-only and do NOT have internal pull-up resistors.
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << LEFT_ENC_A) | (1ULL << LEFT_ENC_B) | (1ULL << RIGHT_ENC_A) | (1ULL << RIGHT_ENC_B),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};

    // Enable internal pull-ups only for pins that support them (GPIO0-33)
    // RIGHT_ENC_A/B (GPIO32/33) may use internal pull-ups
    // LEFT_ENC_A/B (GPIO34/35) are input-only pads and require external pull-ups if needed
    (void)gpio_config(&io_conf);
    (void)gpio_set_pull_mode(RIGHT_ENC_A, GPIO_PULLUP_ONLY);
    (void)gpio_set_pull_mode(RIGHT_ENC_B, GPIO_PULLUP_ONLY);

    // log_isr_error(gpio_install_isr_service(0));
    // Install GPIO ISR service with default configuration
    log_isr_error(gpio_install_isr_service(0));

    add_handler(LEFT_ENC_A, left_enc_cb);
    add_handler(RIGHT_ENC_A, right_enc_cb);
}

void get_robot_state(robot_state_t *copy)
{
    portENTER_CRITICAL(&g_odom.state_mux);
    *copy = g_odom.state; // Thread-safe structural copy
    portEXIT_CRITICAL(&g_odom.state_mux);
}

void update_robot_state()
{
    static long last_l = 0;
    static long last_r = 0;

    portENTER_CRITICAL(&g_odom.tick_mux);
    long curr_l = g_odom.left_ticks;
    long curr_r = g_odom.right_ticks;
    portEXIT_CRITICAL(&g_odom.tick_mux);

    const float meters_per_tick = (2.0F * (float)M_PI * WHEEL_RADIUS) / (float)TICKS_PER_REV;
    float d_l = (float)(curr_l - last_l) * meters_per_tick;
    float d_r = (float)(curr_r - last_r) * meters_per_tick;

    float d_dist = (d_r + d_l) / HALF_DIVISOR;
    float d_theta = (d_r - d_l) / WHEEL_BASE;

    float local_vel_l = d_l / CNT_TS;
    float local_vel_r = d_r / CNT_TS;

    float ma_sin = 0.0F;
    float ma_cos = 0.0F;

    // Update State
    portENTER_CRITICAL(&g_odom.state_mux);

    // Heading used for movement calculations is the average of the start and end headings
    // during the interval, which is more accurate for small rotations
    float move_angle = g_odom.state.theta + (d_theta / HALF_DIVISOR);

    sincosf(move_angle, &ma_sin, &ma_cos);
    g_odom.state.x += d_dist * ma_cos;
    g_odom.state.y += d_dist * ma_sin;
    g_odom.state.theta += d_theta;
    g_odom.state.vel_l = local_vel_l;
    g_odom.state.vel_r = local_vel_r;

    portEXIT_CRITICAL(&g_odom.state_mux);
}
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <uxr/client/client.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>

#define RMW_UXRCE_TRANSPORT_CUSTOM
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            vTaskDelete(NULL);                                                           \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

// Global ROS2 entities
rcl_subscription_t subscriber;
rcl_publisher_t feedback_publisher;
geometry_msgs__msg__Twist msg_twist;
std_msgs__msg__String msg_feedback;
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry msg_odom;

static size_t uart_port = UART_NUM_0;

// ====== Motor Control Functions ======

// Motor pin definitions for ESP32-WROOM-32U (L298 control)
#define LEFT_MOTOR_FORWARD_GPIO 25
#define LEFT_MOTOR_BACKWARD_GPIO 26
#define RIGHT_MOTOR_FORWARD_GPIO 27
#define RIGHT_MOTOR_BACKWARD_GPIO 14

// 4 LEDC Channels
#define L_F_CH LEDC_CHANNEL_0
#define L_B_CH LEDC_CHANNEL_1
#define R_F_CH LEDC_CHANNEL_2
#define R_B_CH LEDC_CHANNEL_3

// LEDC/PWM settings
#define TIM_FREQ_HZ 5000
#define PWM_MAX_DUTY ((1 << 8) - 1) // 255

#define MOTOR_DEADZONE 40 // Minimum PWM to actually move the wheels
#define MOTOR_MAX 255

static inline int clamp_int(int v, int min, int max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

int apply_deadzone(int pwm)
{
    if (pwm == 0)
        return 0;

    // If absolute speed is too low, stop the motor to prevent humming
    if (pwm > -MOTOR_DEADZONE && pwm < MOTOR_DEADZONE)
    {
        return 0;
    }
    return pwm;
}

static void motor_init(void)
{
    // 1. Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = TIM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Helper to configure each channel
    int pins[] = {LEFT_MOTOR_FORWARD_GPIO, LEFT_MOTOR_BACKWARD_GPIO,
                  RIGHT_MOTOR_FORWARD_GPIO, RIGHT_MOTOR_BACKWARD_GPIO};
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

static void set_motor_speeds(int left_pwm, int right_pwm)
{
    // Left Motor Logic
    if (left_pwm > 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, L_F_CH, left_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, L_B_CH, 0);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, L_F_CH, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, L_B_CH, -left_pwm);
    }

    // Right Motor Logic
    if (right_pwm > 0)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, R_F_CH, right_pwm);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, R_B_CH, 0);
    }
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, R_F_CH, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, R_B_CH, -right_pwm);
    }

    // Apply the changes
    ledc_update_duty(LEDC_LOW_SPEED_MODE, L_F_CH);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, L_B_CH);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, R_F_CH);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, R_B_CH);
}

// ====== Odometry Calculations ======

// --- Robot Physical Constants ---
#define WHEEL_RADIUS 0.0325 // meters (e.g., 65mm diameter)
#define WHEEL_BASE 0.125    // meters (distance between wheels)
#define TICKS_PER_REV 1270  // pulses per revolution (4x encoding)

// Encoder Pins
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33
#define LEFT_ENC_A 34
#define LEFT_ENC_B 35

// Robot Pose
double robot_x = 0.0;
double robot_y = 0.0;
double robot_theta = 0.0;
int64_t last_odom_time = 0;
volatile long left_tick_count = 0;
volatile long right_tick_count = 0;

// --- Encoder Interrupt Handlers ---
void IRAM_ATTR left_enc_cb()
{
    int a = gpio_get_level(LEFT_ENC_A);
    int b = gpio_get_level(LEFT_ENC_B);
    if (a == b)
        left_tick_count++;
    else
        left_tick_count--;
}
void IRAM_ATTR right_enc_cb()
{
    int a = gpio_get_level(RIGHT_ENC_A);
    int b = gpio_get_level(RIGHT_ENC_B);
    if (a != b)
        right_tick_count++;
    else
        right_tick_count--;
}

void init_encoders()
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

void update_odometry()
{
    static long last_left_ticks = 0;
    static long last_right_ticks = 0;

    long curr_left = left_tick_count;
    long curr_right = right_tick_count;

    double d_left = (double)(curr_left - last_left_ticks) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);
    double d_right = (double)(curr_right - last_right_ticks) * (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV);

    last_left_ticks = curr_left;
    last_right_ticks = curr_right;

    double d_dist = (d_right + d_left) / 2.0;
    double d_theta = (d_right - d_left) / WHEEL_BASE;

    // Update Pose
    robot_x += d_dist * cos(robot_theta + d_theta / 2.0);
    robot_y += d_dist * sin(robot_theta + d_theta / 2.0);
    robot_theta += d_theta;

    // Fill Message
    msg_odom.pose.pose.position.x = robot_x;
    msg_odom.pose.pose.position.y = robot_y;
    // Quaternion for 2D (Yaw to Quat)
    msg_odom.pose.pose.orientation.z = sin(robot_theta / 2.0);
    msg_odom.pose.pose.orientation.w = cos(robot_theta / 2.0);

    // --- Covariance Matrix (Crucial for SLAM) ---
    // Pose Covariance: x, y, z, roll, pitch, yaw (6x6 matrix)
    // We increase uncertainty as the robot moves.
    for (int i = 0; i < 36; i++)
        msg_odom.pose.covariance[i] = 0.0;
    msg_odom.pose.covariance[0] = 0.001; // X variance
    msg_odom.pose.covariance[7] = 0.001; // Y variance
    msg_odom.pose.covariance[35] = 0.01; // Yaw variance

    // Twist Covariance: linear x, y, z, angular x, y, z
    for (int i = 0; i < 36; i++)
        msg_odom.twist.covariance[i] = 0.0;
    msg_odom.twist.covariance[0] = 0.001;
    msg_odom.twist.covariance[35] = 0.001;

    int64_t time_ms = rmw_uros_epoch_millis();
    msg_odom.header.stamp.sec = (int32_t)(time_ms / 1000);
    msg_odom.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
    msg_odom.header.frame_id.data = "odom";
    msg_odom.header.frame_id.size = strlen("odom");

    RCSOFTCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL));
}

// ================================================================================

void twist_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Prepare and send a feedback string
    char buf[50];
    snprintf(buf, sizeof(buf), "Recv: Lin %.2f, Ang %.2f", msg->linear.x, msg->angular.z);
    msg_feedback.data.data = buf;
    msg_feedback.data.size = strlen(buf);
    RCSOFTCHECK(rcl_publish(&feedback_publisher, &msg_feedback, NULL));

    // Map Twist to motor PWM
    // Simple differential drive mixing: left = linear - angular, right = linear + angular
    // Expectation: incoming linear.x and angular.z are in roughly [-1,1]. Scale to PWM.
    const float SCALE = 255.0f; // scale factor from message units to PWM range
    float left_f = msg->linear.x - msg->angular.z;
    float right_f = msg->linear.x + msg->angular.z;

    int left_pwm = (int)(left_f * SCALE);
    int right_pwm = (int)(right_f * SCALE);

    left_pwm = clamp_int(left_pwm, -PWM_MAX_DUTY, PWM_MAX_DUTY);
    right_pwm = clamp_int(right_pwm, -PWM_MAX_DUTY, PWM_MAX_DUTY);

    set_motor_speeds(apply_deadzone(left_pwm), apply_deadzone(right_pwm));
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Do not start any ROS operations until the micro-ROS agent is available
    while (RCL_RET_OK != rmw_uros_ping_agent(100, 1))
    {
        printf("Waiting for agent...\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Initialize support
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Initialize motors and encoders
    motor_init();
    init_encoders();

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_twist_node", "", &support));

    // 1. Create Subscriber (cmd_vel)
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // 2. Create Publishers
    // twist feedback
    RCCHECK(rclc_publisher_init_default(
        &feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "esp32_twist_feedback"));
    // Odom
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // 3. Create Executor
    // We have 1 handle (the subscriber). Publishers do not need an executor handle.
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &subscriber,
        &msg_twist,
        &twist_cb,
        ON_NEW_DATA));

    // Initialize string message buffer
    msg_feedback.data.capacity = 50;

    // Sync time with the micro-ROS Agent
    rmw_uros_sync_session(1000);

    while (1)
    {
        // 1. Check for incoming Twist messages (handles motor speeds)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // 2. Calculate new Odometry from encoder ticks
        update_odometry();

        // 3. Publish Odometry
        RCSOFTCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL));

        // 4. Control Loop Frequency (e.g., 20ms = 50Hz)
        // This is important for PREEMPT-RT stability on the host side
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Cleanup
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&feedback_publisher, &node));
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize the UART transport for micro-ROS
    // This links the RMW layer to the physical UART pins
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        4000,
        NULL,
        5,
        NULL,
        1 // Pin to Core 1 to keep Core 0 free
    );
}
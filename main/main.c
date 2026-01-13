#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <uxr/client/client.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>

#define RMW_UXRCE_TRANSPORT_CUSTOM
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"
#include "motor_driver.h"
#include "odometry.h"
#include "pid.h"

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

rcl_subscription_t twist_sub;
rcl_publisher_t odom_pub, heartbeat_pub;
geometry_msgs__msg__Twist msg_twist;
nav_msgs__msg__Odometry msg_odom;
std_msgs__msg__Int32 heartbeat_msg;
static int32_t heartbeat_counter = 0;
static int64_t last_cmd_time = 0;
static size_t uart_port = UART_NUM_0;

#define CMD_VEL_TIMEOUT_MS 500
#define ROBOT_WHEEL_BASE 0.125
PID_t pid_l, pid_r;

#include <std_msgs/msg/float32.h>
rcl_publisher_t wheel_left_pub, wheel_right_pub;
std_msgs__msg__Float32 wheel_left_msg_, wheel_right_msg_;

void twist_cb(const void *msgin)
{
    last_cmd_time = esp_timer_get_time() / 1000;
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Target wheel speeds in m/s
    pid_l.setpoint = msg->linear.x - (msg->angular.z * ROBOT_WHEEL_BASE / 2.0);
    pid_r.setpoint = msg->linear.x + (msg->angular.z * ROBOT_WHEEL_BASE / 2.0);
}

void controller_task(void *arg)
{
    // Initialize Feed-forward PID gains: [Kp, Ki, Kd, Kff]
    pid_init(&pid_l, 150.0, 60.0, 10.0, 270.0);
    pid_init(&pid_r, 150.0, 60.0, 10.0, 270.0);

    uint64_t last_time = esp_timer_get_time();
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // Simple Alpha Filter (0.0 to 1.0) - helps smooth out encoder jitter
    float filter_alpha = 0.2;
    float filtered_vel_l = 0;
    float filtered_vel_r = 0;

    while (1)
    {
        // Actual DT
        uint64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) / 1000000.0f; // ms to s
        last_time = now;

        // Prevent division by zero or huge spikes on first run
        if (dt <= 0)
            dt = 0.02;

        update_robot_state(dt);

        // Low-pass filter to measured velocities
        robot_state_t robot_state;
        get_robot_state(&robot_state);
        filtered_vel_l = (filter_alpha * robot_state.vel_l) + (1.0 - filter_alpha) * filtered_vel_l;
        filtered_vel_r = (filter_alpha * robot_state.vel_r) + (1.0 - filter_alpha) * filtered_vel_r;

        wheel_left_msg_.data = filtered_vel_l;
        wheel_right_msg_.data = filtered_vel_r;

        // Watchdog check
        int64_t now_ms = now / 1000;
        if (now_ms - last_cmd_time > CMD_VEL_TIMEOUT_MS)
        {
            // Stop motors if no cmd_vel received recently
            pid_reset(&pid_l);
            pid_reset(&pid_r);
            set_motor_speeds(0, 0);
        }
        else
        {
            // Compute PID and set motor speeds
            int out_l = pid_compute(&pid_l, filtered_vel_l, dt);
            int out_r = pid_compute(&pid_r, filtered_vel_r, dt);
            set_motor_speeds(apply_deadzone(out_l), apply_deadzone(out_r));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz
    }
}

void micro_ros_task(void *arg)
{
    // Initialize micro-ROS support
    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create ROS2 entities
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_controller_node", "", &support));

    RCCHECK(rclc_publisher_init_default(&wheel_left_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "left_wheel_vel"));
    RCCHECK(rclc_publisher_init_default(&wheel_right_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "right_wheel_vel"));

    RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "fastbot_odom"));
    RCCHECK(rclc_publisher_init_default(&heartbeat_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "esp32_heartbeat"));
    RCCHECK(rclc_subscription_init_default(&twist_sub, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

    // Attach to executor (subscriptions only)
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &msg_twist, &twist_cb, ON_NEW_DATA));

    // Wait for agent connection
    // while (RCL_RET_OK != rmw_uros_ping_agent(100, 1))
    // {
    //     printf("Waiting for agent...\n");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Publish heartbeat
        heartbeat_msg.data = heartbeat_counter++;
        RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));

        // Snapshot robot state
        robot_state_t robot_state;
        get_robot_state(&robot_state);

        // Publish odometry
        int64_t time_ms = rmw_uros_epoch_millis();
        msg_odom.header.stamp.sec = (int32_t)(time_ms / 1000);
        msg_odom.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
        msg_odom.header.frame_id.data = "odom";
        msg_odom.pose.pose.position.x = robot_state.x;
        msg_odom.pose.pose.position.y = robot_state.y;
        msg_odom.pose.pose.orientation.z = sin(robot_state.theta / 2.0);
        msg_odom.pose.pose.orientation.w = cos(robot_state.theta / 2.0);
        RCSOFTCHECK(rcl_publish(&odom_pub, &msg_odom, NULL));

        RCSOFTCHECK(rcl_publish(&wheel_left_pub, &wheel_left_msg_, NULL));
        RCSOFTCHECK(rcl_publish(&wheel_right_pub, &wheel_right_msg_, NULL));

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }

    // Cleanup
    RCCHECK(rcl_subscription_fini(&twist_sub, &node));
    RCCHECK(rcl_publisher_fini(&odom_pub, &node));
    RCCHECK(rcl_publisher_fini(&heartbeat_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Initialize micro-ROS custom UART transport
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);

    // Initialize hardware
    configure_motors();
    configure_encoders();

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        4000,
        NULL,
        5, // Lower priority for communication
        NULL,
        0); // On PRO_CPU

    xTaskCreatePinnedToCore(
        controller_task,
        "controller_task",
        4000,
        NULL,
        6, // Higher priority for the PID loop
        NULL,
        1); // On APP_CPU
}
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
static size_t uart_port = UART_NUM_0;
static int64_t last_cmd_time = 0;
#define CMD_VEL_TIMEOUT_MS 500
#define ROBOT_WHEEL_BASE 0.125
PID_t pid_l, pid_r;

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
    // Initialize PIDs: [Kp, Ki, Kd]
    pid_init(&pid_l, 200.0, 100.0, 10.0, -255, 255);
    pid_init(&pid_r, 200.0, 100.0, 10.0, -255, 255);

    const float dt = 0.02; // 20ms (50Hz)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        wheel_vel_t current_vel = get_wheel_velocities(dt);

        // Watchdog check
        int64_t now = esp_timer_get_time() / 1000;
        if (now - last_cmd_time > CMD_VEL_TIMEOUT_MS)
        {
            pid_reset(&pid_l);
            pid_reset(&pid_r);
            set_motor_speeds(0, 0);
        }
        else
        {
            int out_l = pid_compute(&pid_l, current_vel.left, dt);
            int out_r = pid_compute(&pid_r, current_vel.right, dt);

            // Drive motors via your L298 (includes clamping and deadzone)
            set_motor_speeds(apply_deadzone(out_l), apply_deadzone(out_r));
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Wait for agent connection
    while (RCL_RET_OK != rmw_uros_ping_agent(100, 1))
    {
        printf("Waiting for agent...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Initialize hardware
    configure_motors();
    configure_encoders();

    // Create ROS2 entities
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_controller_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&twist_sub, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
    RCCHECK(rclc_publisher_init_default(&heartbeat_pub, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                        "esp32_heartbeat"));

    // Attach to executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &msg_twist, &twist_cb, ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Publish heartbeat
        heartbeat_msg.data = heartbeat_counter++;
        RCSOFTCHECK(rcl_publish(&heartbeat_pub, &heartbeat_msg, NULL));

        update_odometry(&msg_odom);

        // Setup Odom msg header
        int64_t time_ms = rmw_uros_epoch_millis();
        msg_odom.header.stamp.sec = (int32_t)(time_ms / 1000);
        msg_odom.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
        msg_odom.header.frame_id.data = "odom";

        RCSOFTCHECK(rcl_publish(&odom_pub, &msg_odom, NULL));
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

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        4000,
        NULL,
        5,
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
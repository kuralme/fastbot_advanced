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
#include "motor_driver.h"
#include "odometry.h"

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

rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher;
geometry_msgs__msg__Twist msg_twist;
nav_msgs__msg__Odometry msg_odom;

static size_t uart_port = UART_NUM_0;

static inline int clamp_int(int v, int min, int max)
{
    if (v < min)
        return min;
    if (v > max)
        return max;
    return v;
}

void twist_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    const float SCALE = 255.0f;

    int left_pwm = (int)((msg->linear.x - msg->angular.z) * SCALE);
    int right_pwm = (int)((msg->linear.x + msg->angular.z) * SCALE);

    set_motor_speeds(apply_deadzone(left_pwm), apply_deadzone(right_pwm));
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    while (RCL_RET_OK != rmw_uros_ping_agent(100, 1))
    {
        printf("Waiting for agent...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    configure_motors();
    configure_encoders();

    RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                           ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node,
                                        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_twist, &twist_cb, ON_NEW_DATA));

    while (1)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        update_odometry(&msg_odom);

        // Setup Odom msg header
        int64_t time_ms = rmw_uros_epoch_millis();
        msg_odom.header.stamp.sec = (int32_t)(time_ms / 1000);
        msg_odom.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
        msg_odom.header.frame_id.data = "odom";

        rcl_publish(&odom_publisher, &msg_odom, NULL);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Cleanup
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void app_main(void)
{
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
        1);
}
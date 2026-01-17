#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdatomic.h>
#include <uxr/client/client.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
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
#include <std_srvs/srv/trigger.h>

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
rcl_service_t reset_service;
geometry_msgs__msg__Twist msg_twist_;
nav_msgs__msg__Odometry msg_odom_;
std_msgs__msg__Int32 heartbeat_msg_;
std_srvs__srv__Trigger_Request ros_old_req;
std_srvs__srv__Trigger_Response ros_old_res;
static int64_t last_cmd_time_ = 0;
static int64_t last_sync_time_ = 0;
static size_t uart_port = UART_NUM_0;
#define ROBOT_WHEEL_BASE 0.125

PID_t pid_l, pid_r;
#define PID_TS 0.02 // 20 ms
static const float filter_alpha = 0.5;
static float filtered_vel_l = 0;
static float filtered_vel_r = 0;
static int stall_counter = 0;
static bool first_cmd_received = false;
#define STALL_THRESHOLD_MS 200
#define STALL_TICKS (STALL_THRESHOLD_MS / 20) // 10 iterations at 50Hz
#define MIN_SAFE_PWM 50                       // Min PWM should definitely move the robot
#define CMD_VEL_TIMEOUT_MS 300

typedef enum
{
    SYSTEM_OK = 0,
    SYSTEM_FAULT_STALL,
    SYSTEM_FAULT_WATCHDOG
} system_state_t;
atomic_int system_status_ = ATOMIC_VAR_INIT(SYSTEM_OK);

void IRAM_ATTR pid_timer_callback(void *arg)
{
    // Check for faults
    if (atomic_load(&system_status_) != SYSTEM_OK)
    {
        set_motor_speeds(0, 0);
        return;
    }

    robot_state_t state;
    update_robot_state(PID_TS);
    get_robot_state(&state);

    // Simple Alpha Filter (0.0 to 1.0) - helps smooth out encoder jitter
    filtered_vel_l = (filter_alpha * state.vel_l) + (1.0 - filter_alpha) * filtered_vel_l;
    filtered_vel_r = (filter_alpha * state.vel_r) + (1.0 - filter_alpha) * filtered_vel_r;

    // Watchdog & PID Compute
    int64_t now_ms = esp_timer_get_time() / 1000;
    if (!first_cmd_received)
    {
        if (last_cmd_time_ > 0)
        {
            first_cmd_received = true;
        }
        else
        {
            set_motor_speeds(0, 0);
            return;
        }
    }
    if (now_ms - last_cmd_time_ > CMD_VEL_TIMEOUT_MS)
    {
        set_motor_speeds(0, 0);
        pid_reset(&pid_l);
        pid_reset(&pid_r);
        // atomic_store(&system_status_, SYSTEM_FAULT_WATCHDOG);
    }
    else
    {
        int out_l = pid_compute(&pid_l, filtered_vel_l, PID_TS);
        int out_r = pid_compute(&pid_r, filtered_vel_r, PID_TS);

        // --- ENCODER SANITY CHECK ---
        // Significant cmd but near-zero movement
        bool motor_stuck_l = (abs(out_l) > MIN_SAFE_PWM && fabs(state.vel_l) < 0.001);
        bool motor_stuck_r = (abs(out_r) > MIN_SAFE_PWM && fabs(state.vel_r) < 0.001);

        if (motor_stuck_l || motor_stuck_r)
            stall_counter++;
        else
            stall_counter = 0;

        if (stall_counter > STALL_TICKS)
        {
            atomic_store(&system_status_, SYSTEM_FAULT_STALL);
            set_motor_speeds(0, 0);
            return;
        }

        set_motor_speeds(apply_deadzone(out_l), apply_deadzone(out_r));
    }
}

void start_control_loop()
{
    // Initialize Feed-forward PID gains: [Kp, Ki, Kd, Kff]
    pid_init(&pid_l, 150.0, 50.0, 10.0, 18.0);
    pid_init(&pid_r, 150.0, 50.0, 10.0, 18.0);

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &pid_timer_callback,
        .name = "pid_control_loop"};

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 20000)); // 50 Hz (20 ms)
}

void twist_cb(const void *msgin)
{
    last_cmd_time_ = esp_timer_get_time() / 1000;
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Target wheel speeds in m/s
    pid_l.setpoint = msg->linear.x - (msg->angular.z * ROBOT_WHEEL_BASE / 2.0);
    pid_r.setpoint = msg->linear.x + (msg->angular.z * ROBOT_WHEEL_BASE / 2.0);
}

void reset_service_cb(const void *req, void *res)
{
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *)res;

    stall_counter = 0;
    pid_reset(&pid_l);
    pid_reset(&pid_r);
    atomic_store(&system_status_, SYSTEM_OK);

    res_in->success = true;
    res_in->message.data = "Fault Cleared";
    res_in->message.size = strlen(res_in->message.data);
    res_in->message.capacity = res_in->message.size + 1;
}

void destroy_uros_entities(rcl_node_t *node, rclc_executor_t *executor, rcl_publisher_t *odom_pub,
                           rcl_publisher_t *heartbeat_pub, rcl_subscription_t *twist_sub, rcl_service_t *service)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(node->context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(executor);
    rcl_publisher_fini(odom_pub, node);
    rcl_publisher_fini(heartbeat_pub, node);
    rcl_subscription_fini(twist_sub, node);
    rcl_service_fini(service, node);
    rcl_node_fini(node);
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    esp_task_wdt_add(NULL); // Subscribe the task to TWDT

    while (1)
    {
        // --- STATE 1: WAIT FOR AGENT ---
        while (rmw_uros_ping_agent(50, 1) != RCL_RET_OK)
        {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // --- STATE 2: INITIALIZE MICRO-ROS ENTITIES ---
        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        RCCHECK(rclc_node_init_default(&node, "esp32_controller_node", "", &support));

        RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/fastbot/odom"));
        RCCHECK(rclc_publisher_init_default(&heartbeat_pub, &node,
                                            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/fastbot/heartbeat"));
        RCCHECK(rclc_subscription_init_default(&twist_sub, &node,
                                               ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/fastbot/cmd_vel"));
        RCCHECK(rclc_service_init_default(&reset_service, &node,
                                          ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/fastbot/reset_fault"));

        RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
        RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &msg_twist_, &twist_cb, ON_NEW_DATA));
        RCCHECK(rclc_executor_add_service(&executor, &reset_service, &ros_old_req, &ros_old_res, reset_service_cb));

        // --- STATE 3: MAIN LOOP ---
        bool initial_sync_done = false;

        while (1)
        {
            esp_task_wdt_reset();

            if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK)
            {
                break; // Connection lost
            }

            // Handle Time Sync (ESP32-Orange Pi clocks)
            int64_t now_ms = esp_timer_get_time() / 1000;
            if (!initial_sync_done || (now_ms - last_sync_time_ > 30000))
            {
                if (rmw_uros_sync_session(100) == RCL_RET_OK)
                {
                    last_sync_time_ = now_ms;
                    initial_sync_done = true;
                }
            }

            // Process incoming data
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

            // Publish Telemetry
            heartbeat_msg_.data = (int32_t)atomic_load(&system_status_);
            rcl_publish(&heartbeat_pub, &heartbeat_msg_, NULL);

            // Publish Odometry
            robot_state_t robot_state;
            get_robot_state(&robot_state);
            int64_t time_ms = rmw_uros_epoch_millis();

            msg_odom_.header.frame_id.data = (char *)malloc(20);
            strcpy(msg_odom_.header.frame_id.data, "odom");
            msg_odom_.header.frame_id.size = strlen(msg_odom_.header.frame_id.data);
            msg_odom_.header.frame_id.capacity = 20;
            msg_odom_.header.frame_id.data = "odom";
            msg_odom_.header.stamp.sec = (int32_t)(time_ms / 1000);
            msg_odom_.header.stamp.nanosec = (uint32_t)((time_ms % 1000) * 1000000);
            msg_odom_.pose.pose.position.x = robot_state.x;
            msg_odom_.pose.pose.position.y = robot_state.y;
            msg_odom_.pose.pose.orientation.z = sin(robot_state.theta / 2.0);
            msg_odom_.pose.pose.orientation.w = cos(robot_state.theta / 2.0);
            rcl_publish(&odom_pub, &msg_odom_, NULL);

            vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
        }

        // Cleanup before retrying
        destroy_uros_entities(&node, &executor, &odom_pub, &heartbeat_pub, &twist_sub, &reset_service);
        rclc_support_fini(&support);
    }
}

void app_main(void)
{
    // Custom micro-ROS UART transport
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);

    // Configure task watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = (1 << 0) | (1 << 1), // Watch idle tasks on both cores
        .trigger_panic = true,                 // RESET the ESP32 if a task hangs
    };
    esp_task_wdt_init(&twdt_config);

    configure_motors();
    configure_encoders();

    // On hardware timer for deterministic PID loop
    start_control_loop();

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        4000,
        NULL,
        5,
        NULL,
        1); // On APP_CPU
}
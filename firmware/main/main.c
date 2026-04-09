#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdatomic.h>
#include <uxr/client/client.h>

#include "freertos/FreeRTOS.h"
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
#include "common.h"

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

enum
{
    MS_PER_SEC = 1000,
    US_PER_SEC = 1000000,
    NS_PER_MS = 1000000,

    UROS_LOOP_MS = 50,
    STALL_THRESHOLD_MS = 200,
    MIN_VALID_PWM = 45,

    CMD_VEL_TIMEOUT_MS = 500,
    UROS_AGENT_PING_TIMEOUT_MS = 100,
    UROS_SPIN_TIMEOUT_MS = 10,
    UROS_CONNECTION_TIMEOUT_MS = 3000,
    SYNC_TIMEOUT_MS = 30000,
    WATCHDOG_TIMEOUT_MS = 5000
};
typedef enum
{
    SYSTEM_OK = 0,
    SYSTEM_FAULT_STALL,
    SYSTEM_FAULT_WATCHDOG
};

#define FILTER_ALPHA 0.5F
#define STALL_TICKS (STALL_THRESHOLD_MS / 20)            // 10 iterations at 50Hz
#define MIN_VALID_VEL 0.001F                             // For stall detection
static const uint64_t kpid_tim_us = PID_TS * US_PER_SEC; // PID timer in microsec (50Hz)

typedef struct
{
    rclc_support_t support;
    rclc_executor_t executor;
    rcl_node_t node;
    rcl_publisher_t odom_pub;
    rcl_publisher_t heartbeat_pub;
    rcl_subscription_t twist_sub;
    rcl_service_t reset_service;
} uros_entities_t;
typedef struct
{
    PID_t pid_l;
    PID_t pid_r;
    atomic_int status;
    int64_t last_cmd_time;
    int stall_counter;
    bool first_cmd_received;
    float filtered_vel_l;
    float filtered_vel_r;
} robot_manager_t;

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
static robot_manager_t g_bot = {
    .status = ATOMIC_VAR_INIT(SYSTEM_OK),
    .last_cmd_time = 0,
    .stall_counter = 0,
    .first_cmd_received = false,
    .filtered_vel_l = 0.0F,
    .filtered_vel_r = 0.0F,
};

void IRAM_ATTR pid_timer_cb(void *arg __attribute__((unused)))
{
    // Check for faults
    if (g_bot.status != SYSTEM_OK)
    {
        set_motor_speeds(0, 0);
        return;
    }

    if (!g_bot.first_cmd_received)
    {
        set_motor_speeds(0, 0);
        return;
    }

    robot_state_t state;
    update_robot_state();
    get_robot_state(&state);

    // Simple Alpha Filter (0.0 to 1.0) - helps smooth out encoder jitter
    g_bot.filtered_vel_l = (FILTER_ALPHA * state.vel_l) + (1.0F - FILTER_ALPHA) * g_bot.filtered_vel_l;
    g_bot.filtered_vel_r = (FILTER_ALPHA * state.vel_r) + (1.0F - FILTER_ALPHA) * g_bot.filtered_vel_r;

    // --- WATCHDOG ---
    int64_t now_ms = esp_timer_get_time() / MS_PER_SEC;
    if (now_ms - g_bot.last_cmd_time > CMD_VEL_TIMEOUT_MS)
    {
        set_motor_speeds(0, 0);

        if (g_bot.pid_l.setpoint != 0 || g_bot.pid_r.setpoint != 0)
        {
            g_bot.pid_l.setpoint = 0;
            g_bot.pid_r.setpoint = 0;
            pid_reset(&g_bot.pid_l);
            pid_reset(&g_bot.pid_r);
        }
        return;
    }

    // --- PID OUTPUTS ---
    int out_l = pid_compute(&g_bot.pid_l, g_bot.filtered_vel_l);
    int out_r = pid_compute(&g_bot.pid_r, g_bot.filtered_vel_r);

    // --- ENCODER SANITY CHECK ---
    // Significant cmd but near-zero movement
    bool motor_stuck = (abs(out_l) > MIN_VALID_PWM && fabsf(state.vel_l) < MIN_VALID_VEL) ||
                       (abs(out_r) > MIN_VALID_PWM && fabsf(state.vel_r) < MIN_VALID_VEL);

    if (motor_stuck)
    {
        g_bot.stall_counter++;
    }
    else
    {
        g_bot.stall_counter = 0;
    }

    if (g_bot.stall_counter > STALL_TICKS)
    {
        g_bot.status = SYSTEM_FAULT_STALL;
        set_motor_speeds(0, 0);
        return;
    }

    set_motor_speeds(apply_deadzone(out_l), apply_deadzone(out_r));
}

void controller_init(void)
{
    // Initialize Feed-forward PID gains: [Kp, Ki, Kd, Kff]
    pid_init(&g_bot.pid_l, (PID_t){.kp = PID_KP_DEFAULT, .ki = PID_KI_DEFAULT, .kd = PID_KD_DEFAULT, .kff = PID_KFF_DEFAULT});
    pid_init(&g_bot.pid_r, (PID_t){.kp = PID_KP_DEFAULT, .ki = PID_KI_DEFAULT, .kd = PID_KD_DEFAULT, .kff = PID_KFF_DEFAULT});

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &pid_timer_cb,
        .name = "pid_control_loop"};

    esp_timer_handle_t periodic_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, kpid_tim_us));
}

void twist_cb(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    g_bot.last_cmd_time = esp_timer_get_time() / MS_PER_SEC;
    g_bot.first_cmd_received = true;

    // Target wheel speeds in m/s
    g_bot.pid_l.setpoint = (float)msg->linear.x - ((float)msg->angular.z * WHEEL_SEPERATION);
    g_bot.pid_r.setpoint = (float)msg->linear.x + ((float)msg->angular.z * WHEEL_SEPERATION);
}

void reset_robot_manager(void)
{
    g_bot.stall_counter = 0;
    g_bot.first_cmd_received = false;
    g_bot.last_cmd_time = 0;
    g_bot.status = SYSTEM_OK;
    pid_reset(&g_bot.pid_l);
    pid_reset(&g_bot.pid_r);
}
void reset_service_cb(const void *req __attribute__((unused)), void *res)
{
    std_srvs__srv__Trigger_Response *res_in = (std_srvs__srv__Trigger_Response *)res;

    reset_robot_manager();

    res_in->success = true;
    res_in->message.data = "Fault Cleared";
    res_in->message.size = strlen(res_in->message.data);
    res_in->message.capacity = res_in->message.size + 1;
}

void destroy_uros_entities(uros_entities_t *ent)
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context((&ent->node)->context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    (void)rcl_publisher_fini(&ent->odom_pub, &ent->node);
    (void)rcl_publisher_fini(&ent->heartbeat_pub, &ent->node);
    (void)rcl_subscription_fini(&ent->twist_sub, &ent->node);
    (void)rcl_service_fini(&ent->reset_service, &ent->node);
    (void)rcl_node_fini(&ent->node);
    (void)rclc_support_fini(&ent->support);
    (void)rclc_executor_fini(&ent->executor);
}

static bool init_uros_entities(uros_entities_t *ent, rcl_allocator_t *alloc,
                               geometry_msgs__msg__Twist *twist_msg,
                               std_srvs__srv__Trigger_Request *req,
                               std_srvs__srv__Trigger_Response *res)
{
    if (rclc_support_init(&ent->support, 0, NULL, alloc) != RCL_RET_OK)
    {
        return false;
    }
    if (rclc_node_init_default(&ent->node, "fastbot_node", "", &ent->support) != RCL_RET_OK)
    {
        return false;
    }

    // Publishers
    (void)rclc_publisher_init_default(&ent->odom_pub, &ent->node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/fastbot/odom");
    (void)rclc_publisher_init_default(&ent->heartbeat_pub, &ent->node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/fastbot/heartbeat");

    // Subscriptions & Services
    (void)rclc_subscription_init_default(&ent->twist_sub, &ent->node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/fastbot/cmd_vel");
    (void)rclc_service_init_default(&ent->reset_service, &ent->node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/fastbot/reset_fault");

    // Executor
    if (rclc_executor_init(&ent->executor, &ent->support.context, 2, alloc) != RCL_RET_OK)
    {
        return false;
    }
    (void)rclc_executor_add_subscription(&ent->executor, &ent->twist_sub, twist_msg, &twist_cb, ON_NEW_DATA);
    (void)rclc_executor_add_service(&ent->executor, &ent->reset_service, req, res, reset_service_cb);

    return true;
}

static void publish_telemetry(uros_entities_t *ent, nav_msgs__msg__Odometry *msg_odom, std_msgs__msg__Int32 *msg_hb)
{
    robot_state_t state;
    get_robot_state(&state);
    int64_t time_ms = rmw_uros_epoch_millis();

    // Heartbeat
    msg_hb->data = (int32_t)g_bot.status;
    (void)rcl_publish(&ent->heartbeat_pub, msg_hb, NULL);

    // Odometry
    static char odom_frame[] = "odom";
    msg_odom->header.frame_id.data = odom_frame;
    msg_odom->header.frame_id.size = strlen(odom_frame);
    msg_odom->header.frame_id.capacity = sizeof(odom_frame);
    msg_odom->header.stamp.sec = (int32_t)(time_ms / MS_PER_SEC);
    msg_odom->header.stamp.nanosec = (uint32_t)((time_ms % MS_PER_SEC) * NS_PER_MS);
    msg_odom->pose.pose.position.x = state.x;
    msg_odom->pose.pose.position.y = state.y;
    msg_odom->pose.pose.orientation.z = sinf(state.theta / HALF_DIVISOR);
    msg_odom->pose.pose.orientation.w = cosf(state.theta / HALF_DIVISOR);
    (void)rcl_publish(&ent->odom_pub, msg_odom, NULL);
}

void micro_ros_task(void *arg __attribute__((unused)))
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    static geometry_msgs__msg__Twist msg_twist_;
    static nav_msgs__msg__Odometry msg_odom_;
    static std_msgs__msg__Int32 heartbeat_msg_;
    static std_srvs__srv__Trigger_Request req;
    static std_srvs__srv__Trigger_Response res;

    static int64_t last_sync_time_ = 0;
    esp_task_wdt_add(NULL); // Subscribe the task to TWDT

    while (1)
    {
        // --- STATE 1: WAIT FOR THE AGENT ---
        while (rmw_uros_ping_agent(UROS_AGENT_PING_TIMEOUT_MS, 1) != RCL_RET_OK)
        {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // --- STATE 2: INITIALIZE MICRO-ROS ENTITIES ---
        static uros_entities_t uros_ent = {0};
        if (!init_uros_entities(&uros_ent, &allocator, &msg_twist_, &req, &res))
        {
            destroy_uros_entities(&uros_ent);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        bool initial_sync_done = false;
        int64_t last_ping_time = 0;

        // --- STATE 3: MAIN LOOP ---
        while (1)
        {
            esp_task_wdt_reset();
            int64_t now_ms = esp_timer_get_time() / MS_PER_SEC;

            // Monitor connection (Every 3 seconds)
            if (now_ms - last_ping_time > UROS_CONNECTION_TIMEOUT_MS)
            {
                if (rmw_uros_ping_agent(UROS_AGENT_PING_TIMEOUT_MS, 1) != RCL_RET_OK)
                {
                    break;
                }
                last_ping_time = now_ms;
            }

            // Time Sync (Every 30 seconds)
            if (!initial_sync_done || (now_ms - last_sync_time_ > SYNC_TIMEOUT_MS))
            {
                if (rmw_uros_sync_session(UROS_AGENT_PING_TIMEOUT_MS) == RCL_RET_OK)
                {
                    last_sync_time_ = now_ms;
                    initial_sync_done = true;
                }
            }

            rclc_executor_spin_some(&uros_ent.executor, RCL_MS_TO_NS(UROS_SPIN_TIMEOUT_MS));

            // Publish Odom + Heartbeat (20Hz)
            publish_telemetry(&uros_ent, &msg_odom_, &heartbeat_msg_);

            vTaskDelay(pdMS_TO_TICKS(UROS_LOOP_MS));
        }

        // Cleanup before retrying
        destroy_uros_entities(&uros_ent);
    }
}

void app_main(void)
{
    // Custom micro-ROS UART transport
    static size_t uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(
        true,
        (void *)&uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read);

    // Configure task watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MS,
        .idle_core_mask = (1 << 0) | (1 << 1), // Watch idle tasks on both cores
        .trigger_panic = true,                 // RESET the ESP32 if a task hangs
    };
    esp_task_wdt_init(&twdt_config);

    configure_motors();
    configure_encoders();

    // On hardware timer for determinism
    controller_init();

    xTaskCreatePinnedToCore(
        micro_ros_task,
        "uros_task",
        CONFIG_MICRO_ROS_STACK_SIZE,
        NULL,
        CONFIG_MICRO_ROS_TASK_PRIO,
        NULL,
        1); // On APP_CPU
}
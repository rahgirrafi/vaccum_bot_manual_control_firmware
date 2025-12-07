#include "micro_ros_keyboard.hpp"
#include <uros_network_interfaces.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "MICRO_ROS";

// Global micro-ROS objects
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_subscription_t keyboard_sub;
std_msgs__msg__Int8 keyboard_msg;

void keyboard_callback(const void *msgin)
{
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
    int8_t command = msg->data;
    
    ESP_LOGI(TAG, "Received keyboard command: %d", command);
    
    // Update motor command and timestamp
    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motor_command = command;
        last_command_time = xTaskGetTickCount(); // Record when command was received
        xSemaphoreGive(motor_mutex);
        
        // Log the action
        switch (command) {
            case 1:
                ESP_LOGI(TAG, "Command: Rotate clockwise");
                break;
            case -1:
                ESP_LOGI(TAG, "Command: Rotate counter-clockwise");
                break;
            case 0:
                ESP_LOGI(TAG, "Command: Stop motor");
                break;
            default:
                ESP_LOGW(TAG, "Unknown command: %d, stopping motor", command);
                motor_command = 0;
                break;
        }
    } else {
        ESP_LOGW(TAG, "Failed to acquire motor mutex");
    }
}

void micro_ros_init(void)
{
    ESP_LOGI(TAG, "Initializing micro-ROS...");
    
    // Initialize message
    memset(&keyboard_msg, 0, sizeof(keyboard_msg));
    
    // Initialize allocator
    allocator = rcl_get_default_allocator();
    
    // Initialize support
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif
    
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    // Initialize node
    node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, "keyboard_motor_right_arm_controller", "", &support));
    
    // Initialize subscriber
    RCCHECK(rclc_subscription_init_default(
        &keyboard_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/keyboard/group4"
    ));
    
    // Initialize executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &keyboard_sub, 
        &keyboard_msg, 
        &keyboard_callback, 
        ON_NEW_DATA
    ));
    
    ESP_LOGI(TAG, "micro-ROS initialized successfully");
    ESP_LOGI(TAG, "Subscribed to topic: /keyboard/group4");
}

void micro_ros_spin_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS spin task started");
    
    const TickType_t task_period = pdMS_TO_TICKS(100); // 10Hz
    
    while (1) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
        
        if (rc != RCL_RET_OK) {
            static uint32_t error_count = 0;
            error_count++;
            if (error_count % 10 == 0) {
                ESP_LOGW(TAG, "Executor spin failed: %d (error count: %lu)", (int)rc, error_count);
            }
        }
        
        vTaskDelay(task_period);
    }
}

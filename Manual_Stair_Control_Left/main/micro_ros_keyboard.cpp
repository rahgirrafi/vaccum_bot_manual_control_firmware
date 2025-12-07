#include "micro_ros_keyboard.hpp"
#include "as5600.hpp"
#include <uros_network_interfaces.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "MICRO_ROS";



void keyboard_callback_group4(const void *msgin)
{
    const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
    int8_t command = msg->data;
    
    ESP_LOGI(TAG, "Received Group 4 keyboard command: %d", command);
    
    // Update motor 2 command and timestamp
    if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        motor2_command = command;
        last_command_time_motor2 = xTaskGetTickCount();
        xSemaphoreGive(motor_mutex);
        
        // Log the action
        switch (command) {
            case 1:
                ESP_LOGI(TAG, "Motor 4 Command: Rotate clockwise");
                break;
            case -1:
                ESP_LOGI(TAG, "Motor 4 Command: Rotate counter-clockwise");
                break;
            case 0:
                ESP_LOGI(TAG, "Motor 4 Command: Stop motor");
                break;
            default:
                ESP_LOGW(TAG, "Motor 4 Unknown command: %d, stopping motor", command);
                motor2_command = 0;
                break;
        }
    } else {
        ESP_LOGW(TAG, "Failed to acquire motor mutex for Motor 4");
    }
}



void micro_ros_init(void)
{
    ESP_LOGI(TAG, "Initializing micro-ROS...");
    
    // Initialize micro-ROS network mutex (CRITICAL for multi-task WiFi/UDP access)
    microros_network_mutex = xSemaphoreCreateMutex();
    if (microros_network_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create microros_network_mutex");
        return;
    }
    ESP_LOGI(TAG, "micro-ROS network mutex created");
    

    std_msgs__msg__Float32MultiArray__init(&encoder_counts_angel_rpm_msgs);
    std_msgs__msg__Int8__init(&keyboard_msg_group4);
    
    // Initialize encoder message mutex
    encoder_msg_mutex = xSemaphoreCreateMutex();
    if (encoder_msg_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder_msg_mutex");
        return;
    }
    
    // Initialize messages
    memset(&keyboard_msg_group4, 0, sizeof(keyboard_msg_group4));
    
    // Initialize allocator
    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    ESP_LOGI(TAG, "Connecting to micro-ROS agent at %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
#endif
    
    ESP_LOGI(TAG, "About to initialize support with options...");
    rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "rclc_support_init_with_options failed with code %d", (int)rc);
        ESP_LOGE(TAG, "Make sure micro-ROS agent is running at %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        ESP_LOGE(TAG, "Check WiFi connectivity and network configuration");
        fflush(stdout);
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Support initialization successful!");
    
    // Small delay to let agent settle
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize node (shortened name to avoid length limits)
    node = rcl_get_zero_initialized_node();
    rc = rclc_node_init_default(&node, "right_motor", "", &support);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Node initialization failed with code %d - Agent connection issue", (int)rc);
        return;
    }
    ESP_LOGI(TAG, "Node initialized: right_motor");
    
    // Small delay before subscription initialization
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize subscriber for Group 4
    ESP_LOGI(TAG, "Initializing subscription to /keyboard/group4...");
    keyboard_sub_group4 = rcl_get_zero_initialized_subscription();
    rc = rclc_subscription_init_default(
        &keyboard_sub_group4,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/keyboard/group4"
    );
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "Subscription initialization failed with code %d", (int)rc);
        return;
    }
    ESP_LOGI(TAG, "Subscription initialized successfully");
    
    
    // Initialize encoder publisher
    encoder_counts_pub = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_default(
        &encoder_counts_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/encoder_sensors_left"
    ));
    
    ESP_LOGI(TAG, "Encoder publisher initialized on topic: /encoder_sensors_left");
    
    // Initialize executor with 1 subscription
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    
    // Add Group 4 subscription
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &keyboard_sub_group4, 
        &keyboard_msg_group4, 
        &keyboard_callback_group4, 
        ON_NEW_DATA
    ));
    
    ESP_LOGI(TAG, "micro-ROS initialized successfully");
    ESP_LOGI(TAG, "Subscribed to topic: /keyboard/group4");
    ESP_LOGI(TAG, "Publishing AS5600 sensor data to: /as5600_sensors_left");
}

void micro_ros_spin_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS spin task started (handling subscriptions and sensor publishing)");
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(100); // 10Hz
    
    int log_counter = 0;
    
    while (1) {
        
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200));
        if (rc != RCL_RET_OK) {
            if (log_counter % 10 == 0) {
                ESP_LOGW("MICRO_ROS_TASK", "Executor spin failed: %d - Agent may not be available", (int)rc);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        }

        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                rcl_ret_t ret = rcl_publish(&encoder_counts_pub, &encoder_counts_angel_rpm_msgs, NULL);
                if (ret != RCL_RET_OK && log_counter % 10 == 0) {
                    ESP_LOGW(TAG, "Failed to publish encoder data: %d", (int)ret);
                }
            if (log_counter % 50 == 0) {
                ESP_LOGI(TAG, "AS5600 Sensor - Motor1: %.2f rad, %.1f RPM", 
                        encoder_counts_angel_rpm_msgs.data.data[0],
                        encoder_counts_angel_rpm_msgs.data.data[1]);
 
            }
            xSemaphoreGive(encoder_msg_mutex);
        }
        else {
            ESP_LOGW(TAG, "Failed to take encoder_msg_mutex in micro_ros_spin_task");
        }
    
        log_counter++;
        vTaskDelayUntil(&last_wake, task_period);
    }
    
}

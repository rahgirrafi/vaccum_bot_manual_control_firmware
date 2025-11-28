#include "micro_ros_keyboard.hpp"
#include "as5600.hpp"
#include <uros_network_interfaces.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "MICRO_ROS";

// Define global variables declared in global_declarations.hpp
espp::As5600 *g_as5600_0 = nullptr;
SemaphoreHandle_t encoder_msg_mutex = NULL;
std_msgs__msg__Float32MultiArray encoder_counts_angel_rpm_msgs;

// micro-ROS objects
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t encoder_counts_pub;
rcl_publisher_t sensor_publisher;
std_msgs__msg__Float32MultiArray sensor_msg;
SemaphoreHandle_t sensor_msg_mutex = NULL;

// Subscriptions and subscription messages
rcl_subscription_t keyboard_sub_group4;
std_msgs__msg__Int8 keyboard_msg_group4;

// Mutex for protecting micro-ROS network operations (WiFi/UDP)
SemaphoreHandle_t microros_network_mutex;

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
    
    // Initialize sensor message mutex
    sensor_msg_mutex = xSemaphoreCreateMutex();
    if (sensor_msg_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor_msg_mutex");
        return;
    }
    
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
    RCCHECK(rclc_node_init_default(&node, "keyboard_motor_group4_controller", "", &support));
    
    // Initialize subscriber for Group 4
    RCCHECK(rclc_subscription_init_default(
        &keyboard_sub_group4,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "/keyboard/group4"
    ));
    
    // Initialize AS5600 sensor publisher
    sensor_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rclc_publisher_init_default(
        &sensor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/as5600_sensors_left"
    ));
    
    ESP_LOGI(TAG, "AS5600 sensor publisher initialized on topic: /as5600_sensors_left");
    
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
    
    // Allocate memory for sensor message data array
    // Data format: [motor4_angle_rad, motor4_rpm]
    sensor_msg.data.capacity = 2;
    sensor_msg.data.size = 2;
    sensor_msg.data.data = (float*)malloc(sensor_msg.data.capacity * sizeof(float));
    
    if (sensor_msg.data.data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor message");
        vTaskDelete(NULL);
        return;
    }
    
    int log_counter = 0;
    
    while (1) {
        // 1. Handle incoming ROS messages (subscriptions)
        if (xSemaphoreTake(microros_network_mutex, portMAX_DELAY) == pdTRUE) {
            rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            xSemaphoreGive(microros_network_mutex);
            
            if (rc != RCL_RET_OK) {
                static uint32_t error_count = 0;
                error_count++;
                if (error_count % 10 == 0) {
                    ESP_LOGW(TAG, "Executor spin failed: %d (error count: %lu)", (int)rc, error_count);
                }
            }
        } else {
            ESP_LOGW(TAG, "Failed to acquire microros_network_mutex for spin");
        }
        
        // 2. Read AS5600 sensor
        float radians_motor2 = 0.0f;
        float rpm_motor2 = 0.0f;
        
        if (g_as5600_0) {
            radians_motor2 = g_as5600_0->get_radians();
            rpm_motor2 = g_as5600_0->get_rpm();
        }
        
        // 3. Publish sensor data
        if (xSemaphoreTake(sensor_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Populate message data
            sensor_msg.data.data[0] = radians_motor2;  // Motor 4 angle in radians
            sensor_msg.data.data[1] = rpm_motor2;       // Motor 4 RPM
            xSemaphoreGive(sensor_msg_mutex);
            
            // Publish with network mutex protection
            if (xSemaphoreTake(microros_network_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                rcl_ret_t ret = rcl_publish(&sensor_publisher, &sensor_msg, NULL);
                xSemaphoreGive(microros_network_mutex);
                
                if (ret != RCL_RET_OK && log_counter % 10 == 0) {
                    ESP_LOGW(TAG, "Failed to publish sensor data: %d", (int)ret);
                }
            }
        }
        
        // 4. Publish encoder data from encoder_sample_task
        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Publish with network mutex protection
            if (xSemaphoreTake(microros_network_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                rcl_ret_t ret = rcl_publish(&encoder_counts_pub, &encoder_counts_angel_rpm_msgs, NULL);
                xSemaphoreGive(microros_network_mutex);
                
                if (ret != RCL_RET_OK && log_counter % 10 == 0) {
                    ESP_LOGW(TAG, "Failed to publish encoder data: %d", (int)ret);
                }
            }
            xSemaphoreGive(encoder_msg_mutex);
        }
        
        // 5. Throttled logging (every 5 seconds)
        if (log_counter % 50 == 0) {
            ESP_LOGI(TAG, "AS5600 Sensor - Motor4: %.2f rad, %.1f RPM", 
                     radians_motor2, rpm_motor2);
            
            // Log encoder data too
            if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ESP_LOGI(TAG, "Encoder Data - [%.2f, %.1f]",
                    encoder_counts_angel_rpm_msgs.data.data[0],
                    encoder_counts_angel_rpm_msgs.data.data[1]);
                xSemaphoreGive(encoder_msg_mutex);
            }
        }
        
        log_counter++;
        vTaskDelayUntil(&last_wake, task_period);
    }
    
    // Cleanup (never reached)
    free(sensor_msg.data.data);
}

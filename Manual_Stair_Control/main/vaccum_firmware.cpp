/*
ESP32 micro-ROS Keyboard Motor Control

This firmware subscribes to /keyboard/group1 topic and controls a motor using TB6612FNG driver.
Commands: 1 = clockwise, -1 = counter-clockwise, 0 = stop
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <uros_network_interfaces.h>
#include "motor_control.hpp"
#include "micro_ros_keyboard.hpp"

static const char *TAG = "MAIN";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 Keyboard Motor Control Node");
    
    // Initialize network interface for micro-ROS
    esp_err_t ret = uros_network_interface_initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network interface: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Network interface initialized");

    // Initialize motor mutex
    motor_mutex = xSemaphoreCreateMutex();
    if (motor_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create motor mutex");
        return;
    }
    
    // Initialize command timestamp to current time (ensures timeout works from start)
    last_command_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Motor mutex created and timestamp initialized");

    // Initialize motor driver
    motor_driver_init();

    // Initialize micro-ROS
    micro_ros_init();

    // Create motor control task
    BaseType_t task_ret = xTaskCreate(
        motor_control_task,
        "motor_control",
        4096,
        NULL,
        2,
        NULL
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor control task");
        return;
    }

    // Create micro-ROS spin task
    task_ret = xTaskCreate(
        micro_ros_spin_task,
        "micro_ros_spin",
        8192,
        NULL,
        1,
        NULL
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create micro-ROS spin task");
        return;
    }
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "System ready - listening for keyboard commands on /keyboard/group1");
    ESP_LOGI(TAG, "Commands: 1=clockwise, -1=counter-clockwise, 0=stop");
}       



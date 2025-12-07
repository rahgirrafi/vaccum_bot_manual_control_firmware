#pragma once
// Global variable DEFINITIONS - these create the actual storage
// Other .hpp files should declare them as extern

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Define encoder-related global variables
espp::As5600 *g_as5600_0 = nullptr;
std_msgs__msg__Float32MultiArray encoder_counts_angel_rpm_msgs = {};
SemaphoreHandle_t encoder_msg_mutex = NULL;

int8_t motor2_command = 0;
TickType_t last_command_time_motor2 = 0;
SemaphoreHandle_t motor_mutex = NULL;

    
SemaphoreHandle_t microros_network_mutex = NULL;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t encoder_counts_pub;
rcl_subscription_t keyboard_sub_group4;
std_msgs__msg__Int8 keyboard_msg_group4;


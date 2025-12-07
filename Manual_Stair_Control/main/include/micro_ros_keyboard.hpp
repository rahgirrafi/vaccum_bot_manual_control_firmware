#ifndef MICRO_ROS_KEYBOARD_HPP
#define MICRO_ROS_KEYBOARD_HPP

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include "esp_log.h"
#include "motor_control.hpp"


// Error checking macros
#define RCCHECK(fn) { \
  rcl_ret_t temp_rc = (fn); \
  if ((temp_rc) != RCL_RET_OK) { \
    ESP_LOGE("MICRO_ROS", "RCCHECK FAILED %s:%d rc=%d", __FILE__, __LINE__, (int)temp_rc); \
    vTaskDelete(NULL); \
  } \
}

#define RCSOFTCHECK(fn) { \
  rcl_ret_t temp_rc = (fn); \
  if((temp_rc) != RCL_RET_OK){ \
    ESP_LOGW("MICRO_ROS", "RCSOFTCHECK %s:%d rc=%d", __FILE__, __LINE__, (int)temp_rc); \
  } \
}

// Global micro-ROS objects
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_subscription_t keyboard_sub_group2;
extern rcl_subscription_t keyboard_sub_group3;
extern std_msgs__msg__Int8 keyboard_msg_group2;
extern std_msgs__msg__Int8 keyboard_msg_group3;

extern SemaphoreHandle_t microros_network_mutex;

// Function declarations
void micro_ros_init(void);
void micro_ros_spin_task(void *arg);
void keyboard_callback_group2(const void *msgin);
void keyboard_callback_group3(const void *msgin);

#endif // MICRO_ROS_KEYBOARD_HPP

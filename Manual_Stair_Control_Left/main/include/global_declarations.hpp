#pragma once
// This header only declares variables that are shared between encoder.cpp and micro_ros_keyboard.cpp
// The actual definitions are in micro_ros_keyboard.cpp

// Forward declarations
namespace espp {
    class As5600;
}

struct SemaphoreData;  // Forward declare FreeRTOS type
typedef struct SemaphoreData* SemaphoreHandle_t;

struct std_msgs__msg__Float32MultiArray;  // Forward declare ROS2 type

// Single AS5600 encoder
extern espp::As5600 *g_as5600_0; // Encoder on I2C_NUM_1

// Mutex for protecting encoder message access
extern SemaphoreHandle_t encoder_msg_mutex;

// Encoder message storage
extern std_msgs__msg__Float32MultiArray encoder_counts_angel_rpm_msgs;

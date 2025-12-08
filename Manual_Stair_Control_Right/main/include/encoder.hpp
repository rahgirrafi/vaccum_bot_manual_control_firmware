#pragma once
// espp includes for AS5600
#include "i2c.hpp"
#include "butterworth_filter.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <std_msgs/msg/float32_multi_array.h>

// I2C bus configuration for dual AS5600 sensors
// First AS5600 on I2C_NUM_1
#define I2C1_SDA_GPIO GPIO_NUM_21
#define I2C1_SCL_GPIO GPIO_NUM_22

// Second AS5600 on I2C_NUM_0
#define I2C0_SDA_GPIO GPIO_NUM_5
#define I2C0_SCL_GPIO GPIO_NUM_4

// Forward declarations
namespace espp {
    class As5600;
}


extern espp::As5600 *g_as5600_0;
extern std_msgs__msg__Float32MultiArray encoder_counts_angel_rpm_msgs;
extern SemaphoreHandle_t encoder_msg_mutex;

// Function declarations
void encoders_init(void);
void encoder_sample_task(void *arg);


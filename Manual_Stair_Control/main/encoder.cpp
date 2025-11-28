#include "encoder.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"


void encoders_init(void)
{
    // Make two separate I2C instances for the two AS5600 sensors
    // I2C_NUM_1 for first AS5600 sensor
    static espp::I2c i2c1({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)21,  // Standard I2C SDA pin
        .scl_io_num = (gpio_num_t)22,  // Standard I2C SCL pin
    });

    // I2C_NUM_0 for second AS5600 sensor
    static espp::I2c i2c2({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)5,   // Alternative SDA pin
        .scl_io_num = (gpio_num_t)4,   // Alternative SCL pin
    });

    // velocity filters for both sensors
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period1 = 0.01f; // 10ms for sensor 1 (100 Hz)
    static constexpr float encoder_update_period2 = 0.01f; // 10ms for sensor 2 (100 Hz)
    
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter_0(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period1});
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter_1(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period2});
    
    // Filter functions for both sensors
    auto filter_fn_0 = [&filter_0](float raw) -> float { return filter_0.update(raw); };
    auto filter_fn_1 = [&filter_1](float raw) -> float { return filter_1.update(raw); };

    // Create first AS5600 on I2C_NUM_1
    g_as5600_0 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c1, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn_0,
         .update_period = std::chrono::duration<float>(encoder_update_period1),
         .log_level = espp::Logger::Verbosity::WARN});

    // Create second AS5600 on I2C_NUM_0
    g_as5600_1 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c2, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn_1,
         .update_period = std::chrono::duration<float>(encoder_update_period2),
         .log_level = espp::Logger::Verbosity::WARN});

    ESP_LOGI("ENCODER", "Dual AS5600 encoders initialized on separate I2C buses");
    ESP_LOGI("ENCODER", "  AS5600_0: I2C_NUM_1 (SDA=21, SCL=22)");
    ESP_LOGI("ENCODER", "  AS5600_1: I2C_NUM_0 (SDA=5, SCL=4)");

   
    ESP_LOGI("ENCODER", "All encoder interrupts configured successfully");
}



// Encoder sample & debug task
void encoder_sample_task(void *arg)
{
    encoders_init();
    (void)arg;
    
    // Initialize the Float32MultiArray message
    encoder_counts_angel_rpm_msgs.data.capacity = 4;
    encoder_counts_angel_rpm_msgs.data.size = 4;
    encoder_counts_angel_rpm_msgs.data.data = (float*)malloc(4 * sizeof(float));
    
    if (encoder_counts_angel_rpm_msgs.data.data == NULL) {
        ESP_LOGE("ENCODER_TASK", "Failed to allocate memory for encoder message");
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize values to zero
    for (int i = 0; i < 4; i++) {
        encoder_counts_angel_rpm_msgs.data.data[i] = 0.0f;
    }
    
    int log_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t sample_period = pdMS_TO_TICKS(100);  // 100ms = 10Hz
    
    ESP_LOGI("ENCODER_TASK", "Encoder sample task started with Float32MultiArray");
    
    while (1) {
        // Read from first AS5600 on I2C_NUM_1
        float radians_0 = 0.0f, rpm_0 = 0.0f;
        if (g_as5600_0) {
            radians_0 = g_as5600_0->get_radians();
            rpm_0 = g_as5600_0->get_rpm();
        }

        // Read from second AS5600 on I2C_NUM_0
        float radians_1 = 0.0f, rpm_1 = 0.0f;
        if (g_as5600_1) {
            radians_1 = g_as5600_1->get_radians();
            rpm_1 = g_as5600_1->get_rpm();
        }

        // Update encoder message with mutex protection
        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Update encoder message with dual AS5600 data
            // Using indices 0-3 for the two AS5600 sensors (radians and RPM for each)
            encoder_counts_angel_rpm_msgs.data.data[0] = radians_0;  // First AS5600 radians
            encoder_counts_angel_rpm_msgs.data.data[1] = rpm_0;      // First AS5600 RPM
            encoder_counts_angel_rpm_msgs.data.data[2] = radians_1;  // Second AS5600 radians
            encoder_counts_angel_rpm_msgs.data.data[3] = rpm_1;      // Second AS5600 RPM
            
            xSemaphoreGive(encoder_msg_mutex);
        } else {
            ESP_LOGW("ENCODER_TASK", "Failed to take encoder_msg_mutex");
        }
            
        // Throttled logging (every 5 seconds at 10Hz)
        if (log_counter % 50 == 0) {
            ESP_LOGI("ENCODER", "AS5600 Sensors - Sensor0: %.2f rad, %.1f RPM | Sensor1: %.2f rad, %.1f RPM", 
                    radians_0, rpm_0, radians_1, rpm_1);
        }
        
        log_counter++;
        vTaskDelayUntil(&last_wake, sample_period);
    }
}

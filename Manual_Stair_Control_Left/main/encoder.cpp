#include "encoder.hpp"
#include "as5600.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"



void encoders_init(void)
{
    // Make one I2C instance for the AS5600 sensor
    // I2C_NUM_1 for AS5600 sensor
    static espp::I2c i2c1({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)22,  // Standard I2C SDA pin
        .scl_io_num = (gpio_num_t)21,  // Standard I2C SCL pin
    });

    // velocity filter
    static constexpr float filter_cutoff_hz = 4.0f;
    static constexpr float encoder_update_period = 0.01f; // 10ms (100 Hz)
    
    static espp::ButterworthFilter<2, espp::BiquadFilterDf2> filter_0(
        {.normalized_cutoff_frequency = 2.0f * filter_cutoff_hz * encoder_update_period});
    
    // Filter function
    auto filter_fn_0 = [&filter_0](float raw) -> float { return filter_0.update(raw); };

    // Create AS5600 on I2C_NUM_1
    g_as5600_0 = new espp::As5600(
        {.write_then_read =
             std::bind(&espp::I2c::write_read, &i2c1, std::placeholders::_1, std::placeholders::_2,
                       std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
         .velocity_filter = filter_fn_0,
         .update_period = std::chrono::duration<float>(encoder_update_period),
         .log_level = espp::Logger::Verbosity::WARN});

    ESP_LOGI("ENCODER", "AS5600 encoder initialized on I2C bus");
    ESP_LOGI("ENCODER", "  AS5600_0: I2C_NUM_1 (SDA=21, SCL=22)");

        // Initialize encoder message array before creating publisher
    encoder_counts_angel_rpm_msgs.data.capacity = 2;
    encoder_counts_angel_rpm_msgs.data.size = 2;
    encoder_counts_angel_rpm_msgs.data.data = (float*)malloc(2 * sizeof(float));
    if (encoder_counts_angel_rpm_msgs.data.data == NULL) {
        ESP_LOGE("Encoder", "Failed to allocate encoder message array");
        return;
    }
    encoder_counts_angel_rpm_msgs.data.data[0] = 0.0f;
    encoder_counts_angel_rpm_msgs.data.data[1] = 0.0f;
    ESP_LOGI("Encoder", "Encoder message array initialized");
}



// Encoder sample & debug task
void encoder_sample_task(void *arg)
{
    encoders_init();
    (void)arg;
    
    // Note: encoder_counts_angel_rpm_msgs is now initialized in micro_ros_init()
    // to ensure it's ready before the publisher is created
    
    int log_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t sample_period = pdMS_TO_TICKS(100);  // 100ms = 10Hz
    
    ESP_LOGI("ENCODER_TASK", "Encoder sample task started with Float32MultiArray");

    float radians_0 = 0.0f, rpm_0 = 0.0f;

    while (1) {

        if (g_as5600_0) {
            radians_0 = g_as5600_0->get_radians();
            rpm_0 = g_as5600_0->get_rpm();
        }

        // Update encoder message with mutex protection
        if (xSemaphoreTake(encoder_msg_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Update encoder message with AS5600 data
            encoder_counts_angel_rpm_msgs.data.data[0] = radians_0;  // AS5600 radians
            encoder_counts_angel_rpm_msgs.data.data[1] = rpm_0;      // AS5600 RPM
            
            xSemaphoreGive(encoder_msg_mutex);
        } else {
            ESP_LOGW("ENCODER_TASK", "Failed to take encoder_msg_mutex in Encoder Sample Task");
        }
            
        // Throttled logging (every 5 seconds at 10Hz)
        // if (log_counter % 50 == 0) {
            ESP_LOGI("ENCODER", "AS5600 Sensor (Motor 4) - %.2f rad, %.1f RPM", 
                    radians_0, rpm_0);
        // }
        
        log_counter++;
        vTaskDelayUntil(&last_wake, sample_period);
    }
}

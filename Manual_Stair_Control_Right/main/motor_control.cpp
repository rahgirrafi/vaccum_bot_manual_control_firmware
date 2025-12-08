#include "motor_control.hpp"

static const char *TAG = "MOTOR_CONTROL";



void motor_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing TB6612FNG motor driver for Motor 4...");
    
    // Configure GPIO pins for Motor 2
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR2_IN1_GPIO) | (1ULL << MOTOR2_IN2_GPIO) | (1ULL << MOTOR2_STBY_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // Set initial states for Motor 2
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);
    gpio_set_level(MOTOR2_STBY_GPIO, 1);  // Enable motor driver
    
    // Configure PWM Timer for Motor 2
    ledc_timer_config_t ledc_timer2 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = MOTOR2_PWM_TIMER,
        .freq_hz = MOTOR2_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer for Motor 2: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure PWM Channel for Motor 2
    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = MOTOR2_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR2_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR2_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ledc_channel2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel for Motor 2: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Motor driver initialized successfully for Motor 4");
}

// Motor 4 control functions
void motor2_rotate_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 4 rotating clockwise");
    
    // Set direction: IN1=1, IN2=0 for clockwise
    gpio_set_level(MOTOR2_IN1_GPIO, 1);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, MOTOR2_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}

void motor2_rotate_counter_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 4 rotating counter-clockwise");
    
    // Set direction: IN1=0, IN2=1 for counter-clockwise
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 1);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, MOTOR2_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}

void motor2_stop(void)
{
    ESP_LOGI(TAG, "Motor 4 stopped");
    
    // Stop motor: IN1=0, IN2=0
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);
    
    // Set PWM duty cycle to 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}



void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "Motor control task started");
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(50); // 20Hz control loop
    
    while (1) {
        int8_t current_command_motor2 = 0;
        TickType_t current_time = xTaskGetTickCount();
        TickType_t command_time_motor2 = 0;
        
        // Get current motor command and timestamp
        if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_command_motor2 = motor2_command;
            command_time_motor2 = last_command_time_motor2;
            xSemaphoreGive(motor_mutex);
        }
        
        // Process Motor 2
        TickType_t time_since_command_motor2 = current_time - command_time_motor2;
        bool command_timeout_motor2 = (time_since_command_motor2 > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS));
        
        if (command_timeout_motor2 && current_command_motor2 != 0) {
            ESP_LOGI(TAG, "Motor 4 command timeout (%.1f ms), stopping motor", 
                     (float)time_since_command_motor2 * portTICK_PERIOD_MS);
            motor2_stop();
            
            if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                motor2_command = 0;
                xSemaphoreGive(motor_mutex);
            }
        } else {
            switch (current_command_motor2) {
                case 1:
                    motor2_rotate_clockwise();
                    break;
                case -1:
                    motor2_rotate_counter_clockwise();
                    break;
                case 0:
                default:
                    motor2_stop();
                    break;
            }
        }
        
        vTaskDelayUntil(&last_wake, task_period);
    }
}

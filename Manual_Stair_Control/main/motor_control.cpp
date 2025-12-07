#include "motor_control.hpp"

static const char *TAG = "MOTOR_CONTROL";

// Global variables
int8_t motor_command = 0;
SemaphoreHandle_t motor_mutex;
TickType_t last_command_time = 0;

void motor_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing TB6612FNG motor driver...");
    
    // Configure GPIO pins for motor driver
    gpio_config_t io_conf = {};
    
    // Configure IN1, IN2, and STBY pins as output
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO) | (1ULL << MOTOR_STBY_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // Set initial states
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    gpio_set_level(MOTOR_STBY_GPIO, 1);  // Enable motor driver
    
    // Configure PWM for motor speed control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = MOTOR_PWM_TIMER,
        .freq_hz = MOTOR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Motor driver initialized successfully");
}

void motor_rotate_clockwise(void)
{
    ESP_LOGI(TAG, "Motor rotating clockwise");
    
    // Set direction: IN1=1, IN2=0 for clockwise
    gpio_set_level(MOTOR_IN1_GPIO, 1);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL, MOTOR_PWM_DUTY / 2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

void motor_rotate_counter_clockwise(void)
{
    ESP_LOGI(TAG, "Motor rotating counter-clockwise");
    
    // Set direction: IN1=0, IN2=1 for counter-clockwise
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 1);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL, MOTOR_PWM_DUTY / 2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

void motor_stop(void)
{
    ESP_LOGI(TAG, "Motor stopped");
    
    // Stop motor: IN1=0, IN2=0
    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    
    // Set PWM duty cycle to 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "Motor control task started");
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(50); // 20Hz control loop
    
    while (1) {
        int8_t current_command = 0;
        TickType_t current_time = xTaskGetTickCount();
        TickType_t command_time = 0;
        
        // Get current motor command and timestamp
        if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_command = motor_command;
            command_time = last_command_time;
            xSemaphoreGive(motor_mutex);
        }
        
        // Check if command has timed out (0.5 seconds)
        TickType_t time_since_command = current_time - command_time;
        bool command_timeout = (time_since_command > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS));
        
        // Execute motor command or stop due to timeout
        if (command_timeout && current_command != 0) {
            // Command timed out, force stop
            ESP_LOGI(TAG, "Motor command timeout (%.1f ms), stopping motor", 
                     (float)time_since_command * portTICK_PERIOD_MS);
            motor_stop();
            
            // Clear the command to prevent repeated timeout messages
            if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                motor_command = 0;
                xSemaphoreGive(motor_mutex);
            }
        } else {
            // Execute normal motor command
            switch (current_command) {
                case 1:
                    motor_rotate_clockwise();
                    break;
                case -1:
                    motor_rotate_counter_clockwise();
                    break;
                case 0:
                default:
                    motor_stop();
                    break;
            }
        }
        
        vTaskDelayUntil(&last_wake, task_period);
    }
}

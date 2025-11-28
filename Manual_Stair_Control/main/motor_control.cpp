#include "motor_control.hpp"

static const char *TAG = "MOTOR_CONTROL";

// Global variables
int8_t motor2_command = 0;
int8_t motor3_command = 0;
SemaphoreHandle_t motor_mutex;
TickType_t last_command_time_motor2 = 0;
TickType_t last_command_time_motor3 = 0;

void motor_driver_init(void)
{
    ESP_LOGI(TAG, "Initializing TB6612FNG motor drivers for Motor 2 and Motor 3...");
    
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
    
    // Configure GPIO pins for Motor 3
    io_conf.pin_bit_mask = (1ULL << MOTOR3_IN1_GPIO) | (1ULL << MOTOR3_IN2_GPIO) | (1ULL << MOTOR3_STBY_GPIO);
    gpio_config(&io_conf);
    
    // Set initial states for Motor 3
    gpio_set_level(MOTOR3_IN1_GPIO, 0);
    gpio_set_level(MOTOR3_IN2_GPIO, 0);
    gpio_set_level(MOTOR3_STBY_GPIO, 1);  // Enable motor driver
    
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
    
    // Configure PWM Timer for Motor 3
    ledc_timer_config_t ledc_timer3 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = MOTOR3_PWM_TIMER,
        .freq_hz = MOTOR3_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    ret = ledc_timer_config(&ledc_timer3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer for Motor 3: %s", esp_err_to_name(ret));
        return;
    }
    
    // Configure PWM Channel for Motor 3
    ledc_channel_config_t ledc_channel3 = {
        .gpio_num = MOTOR3_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = MOTOR3_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = MOTOR3_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&ledc_channel3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel for Motor 3: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Motor drivers initialized successfully for Motor 2 and Motor 3");
}

// Motor 2 control functions
void motor2_rotate_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 2 rotating clockwise");
    
    // Set direction: IN1=1, IN2=0 for clockwise
    gpio_set_level(MOTOR2_IN1_GPIO, 1);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, MOTOR2_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}

void motor2_rotate_counter_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 2 rotating counter-clockwise");
    
    // Set direction: IN1=0, IN2=1 for counter-clockwise
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 1);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, MOTOR2_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}

void motor2_stop(void)
{
    ESP_LOGI(TAG, "Motor 2 stopped");
    
    // Stop motor: IN1=0, IN2=0
    gpio_set_level(MOTOR2_IN1_GPIO, 0);
    gpio_set_level(MOTOR2_IN2_GPIO, 0);
    
    // Set PWM duty cycle to 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR2_PWM_CHANNEL);
}

// Motor 3 control functions
void motor3_rotate_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 3 rotating clockwise");
    
    // Set direction: IN1=1, IN2=0 for clockwise
    gpio_set_level(MOTOR3_IN1_GPIO, 1);
    gpio_set_level(MOTOR3_IN2_GPIO, 0);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL, MOTOR3_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL);
}

void motor3_rotate_counter_clockwise(void)
{
    ESP_LOGI(TAG, "Motor 3 rotating counter-clockwise");
    
    // Set direction: IN1=0, IN2=1 for counter-clockwise
    gpio_set_level(MOTOR3_IN1_GPIO, 0);
    gpio_set_level(MOTOR3_IN2_GPIO, 1);
    
    // Set PWM duty cycle (50% speed)
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL, MOTOR3_PWM_DUTY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL);
}

void motor3_stop(void)
{
    ESP_LOGI(TAG, "Motor 3 stopped");
    
    // Stop motor: IN1=0, IN2=0
    gpio_set_level(MOTOR3_IN1_GPIO, 0);
    gpio_set_level(MOTOR3_IN2_GPIO, 0);
    
    // Set PWM duty cycle to 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR3_PWM_CHANNEL);
}

void motor_control_task(void *arg)
{
    ESP_LOGI(TAG, "Motor control task started");
    
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t task_period = pdMS_TO_TICKS(50); // 20Hz control loop
    
    while (1) {
        int8_t current_command_motor2 = 0;
        int8_t current_command_motor3 = 0;
        TickType_t current_time = xTaskGetTickCount();
        TickType_t command_time_motor2 = 0;
        TickType_t command_time_motor3 = 0;
        
        // Get current motor commands and timestamps
        if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            current_command_motor2 = motor2_command;
            current_command_motor3 = motor3_command;
            command_time_motor2 = last_command_time_motor2;
            command_time_motor3 = last_command_time_motor3;
            xSemaphoreGive(motor_mutex);
        }
        
        // Process Motor 2
        TickType_t time_since_command_motor2 = current_time - command_time_motor2;
        bool command_timeout_motor2 = (time_since_command_motor2 > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS));
        
        if (command_timeout_motor2 && current_command_motor2 != 0) {
            ESP_LOGI(TAG, "Motor 2 command timeout (%.1f ms), stopping motor", 
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
        
        // Process Motor 3
        TickType_t time_since_command_motor3 = current_time - command_time_motor3;
        bool command_timeout_motor3 = (time_since_command_motor3 > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS));
        
        if (command_timeout_motor3 && current_command_motor3 != 0) {
            ESP_LOGI(TAG, "Motor 3 command timeout (%.1f ms), stopping motor", 
                     (float)time_since_command_motor3 * portTICK_PERIOD_MS);
            motor3_stop();
            
            if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                motor3_command = 0;
                xSemaphoreGive(motor_mutex);
            }
        } else {
            switch (current_command_motor3) {
                case 1:
                    motor3_rotate_clockwise();
                    break;
                case -1:
                    motor3_rotate_counter_clockwise();
                    break;
                case 0:
                default:
                    motor3_stop();
                    break;
            }
        }
        
        vTaskDelayUntil(&last_wake, task_period);
    }
}

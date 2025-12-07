#pragma once
#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// TB6612FNG Motor Driver GPIO pins for Motor 4 (Group 4)
#define MOTOR2_PWM_GPIO    GPIO_NUM_26    // PWM signal  
#define MOTOR2_IN1_GPIO    GPIO_NUM_4     // Direction control 1
#define MOTOR2_IN2_GPIO    GPIO_NUM_2     // Direction control 2
#define MOTOR2_STBY_GPIO   GPIO_NUM_22    // Standby pin

// PWM Configuration for Motor 4
#define MOTOR2_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR2_PWM_TIMER   LEDC_TIMER_0
#define MOTOR2_PWM_FREQ    1000           // 1kHz PWM frequency
#define MOTOR2_PWM_DUTY    4095           // 12-bit resolution (0-4095)

// Include global declarations for motor-related shared variables

extern int8_t motor2_command;
extern TickType_t last_command_time_motor2;
extern SemaphoreHandle_t motor_mutex;

// Motor timeout configuration
#define MOTOR_TIMEOUT_MS 500  // 0.5 seconds timeout

// Function declarations
void motor_driver_init(void);
void motor2_rotate_clockwise(void);
void motor2_rotate_counter_clockwise(void);
void motor2_stop(void);
void motor_control_task(void *arg);

#endif // MOTOR_CONTROL_HPP

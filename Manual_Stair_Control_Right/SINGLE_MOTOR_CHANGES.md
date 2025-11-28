# Single Motor and Single Sensor Modifications - Group 4

## Summary
The code has been modified to operate with only one motor (Motor 4 / Group 4) and one AS5600 sensor instead of two motors and two sensors.

## Changes Made

### 1. Main Firmware (`vaccum_firmware.cpp`)
- Updated comments to reflect single motor operation
- Removed Motor 3 timestamp initialization
- Updated log messages to reference only Group 4

### 2. Motor Control (`motor_control.cpp` and `motor_control.hpp`)
- **Removed Motor 3 variables:**
  - `motor3_command`
  - `last_command_time_motor3`
- **Removed Motor 3 functions:**
  - `motor3_rotate_clockwise()`
  - `motor3_rotate_counter_clockwise()`
  - `motor3_stop()`
- **Removed Motor 3 GPIO and PWM configurations:**
  - GPIO pins (IN1, IN2, STBY)
  - PWM timer and channel setup
- **Simplified `motor_control_task()`:**
  - Only processes Motor 2 commands
  - Removed Motor 3 processing logic

### 3. Micro-ROS Keyboard (`micro_ros_keyboard.cpp`)
- **Removed Motor 3 subscription:**
  - Removed `keyboard_sub_group3`
  - Removed `keyboard_msg_group3`
  - Removed `keyboard_callback_group3()` function
- **Updated executor:**
  - Changed from 2 subscriptions to 1 subscription
  - Only handles `/keyboard/group4` topic
- **Updated node name:**
  - Changed from `keyboard_motor_group2_group3_controller` to `keyboard_motor_group4_controller`
- **Updated sensor publishing:**
  - Changed sensor message from 4 floats to 2 floats
  - Format: [motor4_angle_rad, motor4_rpm]
  - Removed Motor 3 sensor reading and publishing
- **Updated logging:**
  - Reports only Motor 4 sensor data

### 4. Encoder (`encoder.cpp`)
- **Removed second AS5600 sensor:**
  - Removed `g_as5600_1` and its I2C bus (I2C_NUM_0)
  - Removed second filter and filter function
- **Simplified encoder initialization:**
  - Only initializes one AS5600 on I2C_NUM_1 (SDA=21, SCL=22)
- **Updated `encoder_sample_task()`:**
  - Changed message array from 4 floats to 2 floats
  - Format: [radians, rpm]
  - Only reads from one sensor
  - Updated logging to show single sensor data

### 5. Global Declarations (`global_declarations.hpp`)
- Removed `g_as5600_1` extern declaration
- Updated comments to reflect single encoder

### 6. Micro-ROS Keyboard (`micro_ros_keyboard.cpp` - globals)
- Removed `g_as5600_1` definition

## Hardware Configuration

### Active Motor (Motor 4 / Group 4):
- **GPIO Pins:**
  - PWM: GPIO 26
  - IN1: GPIO 4
  - IN2: GPIO 2
  - STBY: GPIO 22
- **PWM Settings:**
  - Channel: LEDC_CHANNEL_0
  - Timer: LEDC_TIMER_0
  - Frequency: 1 kHz
  - Duty: 4095 (12-bit, full speed)

### Active Sensor (AS5600):
- **I2C Bus:** I2C_NUM_1
- **GPIO Pins:**
  - SDA: GPIO 21
  - SCL: GPIO 22

## ROS Topics

### Subscriptions:
- `/keyboard/group4` - Int8 messages (1 = clockwise, -1 = counter-clockwise, 0 = stop)

### Publications:
- `/as5600_sensors_left` - Float32MultiArray [angle_radians, rpm]
- `/encoder_sensors_left` - Float32MultiArray [angle_radians, rpm]

## Operation
The system now:
1. Subscribes to only one keyboard command topic (`/keyboard/group4`)
2. Controls only one motor (Motor 4 / Group 4) with TB6612FNG driver
3. Reads from only one AS5600 magnetic encoder sensor
4. Publishes sensor data for one motor at 10 Hz
5. Implements the same timeout safety feature (500ms) for the motor

All functionality remains the same, just reduced from 2 motors/sensors to 1 motor/sensor, and configured for Group 4.

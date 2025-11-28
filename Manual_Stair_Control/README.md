# ESP32 Keyboard Motor Control Firmware

This firmware creates a clean, simple motor control system that subscribes to the `/keyboard/group1` topic and controls a motor using a TB6612FNG motor driver based on the received commands.

## Features

- **Clean Architecture**: Modular design with separate files for motor control and micro-ROS functionality
- **Simple Control Logic**: 
  - Command `1`: Rotate motor clockwise
  - Command `-1`: Rotate motor counter-clockwise  
  - Command `0`: Stop motor
- **Safety Timeout**: Motor automatically stops after 0.5 seconds if no new command is received
- **TB6612FNG Driver Support**: Proper GPIO configuration for the TB6612FNG motor driver
- **Micro-ROS Integration**: Subscribes to `/keyboard/group1` topic for Int8 messages

## Hardware Configuration

### TB6612FNG Motor Driver Connections (Current Placeholders)
```
Motor Driver Pin    ESP32 GPIO    Function
AIN1               GPIO_19       Direction control 1
AIN2               GPIO_21       Direction control 2  
PWMA               GPIO_18       PWM speed control
STBY               GPIO_22       Standby (always high)
```

**Note**: These are placeholder GPIO numbers. Update the `motor_control.hpp` file with your actual pin connections.

## File Structure

```
main/
├── include/
│   ├── motor_control.hpp      # Motor driver definitions and function declarations
│   └── micro_ros_keyboard.hpp # Micro-ROS subscriber definitions
├── motor_control.cpp          # TB6612FNG motor driver implementation
├── micro_ros_keyboard.cpp     # Micro-ROS subscriber and callbacks
└── vaccum_firmware.cpp        # Main application entry point
```

## Key Functions

### Motor Control (`motor_control.cpp`)
- `motor_driver_init()`: Initialize TB6612FNG driver GPIO and PWM
- `motor_rotate_clockwise()`: Set motor to rotate clockwise at 50% speed
- `motor_rotate_counter_clockwise()`: Set motor to rotate counter-clockwise at 50% speed
- `motor_stop()`: Stop motor completely
- `motor_control_task()`: Main motor control task with 0.5s timeout safety (20Hz control loop)

### Micro-ROS (`micro_ros_keyboard.cpp`)
- `micro_ros_init()`: Initialize micro-ROS node and subscriber
- `keyboard_callback()`: Handle incoming Int8 messages from `/keyboard/group1`
- `micro_ros_spin_task()`: Micro-ROS event loop (10Hz)

## Usage

1. **Update GPIO pins**: Modify the `#define` statements in `motor_control.hpp` to match your hardware connections

2. **Build and flash**:
   ```bash
   idf.py build
   idf.py flash
   ```

3. **Run micro-ROS agent** on your computer:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

4. **Send commands** via ROS2:
   ```bash
   # Rotate clockwise
   ros2 topic pub /keyboard/group1 std_msgs/msg/Int8 "data: 1"
   
   # Rotate counter-clockwise  
   ros2 topic pub /keyboard/group1 std_msgs/msg/Int8 "data: -1"
   
   # Stop motor
   ros2 topic pub /keyboard/group1 std_msgs/msg/Int8 "data: 0"
   ```

## Network Configuration

The firmware connects to micro-ROS agent via UDP. Configure your WiFi credentials and micro-ROS agent IP in the ESP-IDF configuration:

```bash
idf.py menuconfig
```

Navigate to `Component config` → `micro-ROS` to set:
- Agent IP address
- Agent port (default: 8888)
- WiFi credentials

## Customization

- **Motor Speed**: Modify the PWM duty cycle values in motor control functions (currently 50%)
- **Control Frequency**: Adjust `task_period` in motor_control_task (currently 20Hz)  
- **Timeout Duration**: Change `MOTOR_TIMEOUT_MS` in `motor_control.hpp` (currently 500ms)
- **GPIO Pins**: Update the `#define` statements in `motor_control.hpp`
- **Topic Name**: Change `/keyboard/group1` in `micro_ros_keyboard.cpp`

## Safety Features

- Mutex protection for motor command variable
- Proper error handling for GPIO and PWM initialization
- Background task isolation between motor control and micro-ROS
- Standby pin always enabled on TB6612FNG
- **Automatic timeout stop**: Motor stops after 0.5 seconds of no commands for safety
- Command timestamp tracking to prevent runaway motor operation

The firmware is designed to be simple, reliable, and easily customizable for different hardware configurations.

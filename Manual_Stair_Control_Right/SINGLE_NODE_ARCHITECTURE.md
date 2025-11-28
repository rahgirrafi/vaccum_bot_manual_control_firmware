# Consolidated Single-Node Architecture

## Overview
Consolidated the micro-ROS implementation into a single node (`keyboard_motor_group2_group3_controller`) that handles both keyboard command subscriptions and AS5600 sensor publishing.

## Changes Made

### Removed Separate Sensor Task
**Before:** Had two separate tasks:
- `micro_ros_spin_task` - Handled keyboard subscriptions
- `sensor_publish_task` - Published AS5600 sensor data

**After:** Single unified task:
- `micro_ros_spin_task` - Handles **both** subscriptions and sensor publishing

### Benefits

1. **Simplified Architecture**
   - One ROS node instead of conceptually separate tasks
   - Cleaner code organization
   - Easier to understand and maintain

2. **Better Resource Usage**
   - Eliminated one FreeRTOS task (saves ~4KB stack)
   - Reduced task switching overhead
   - Single mutex for all network operations

3. **Consistent Timing**
   - Both operations run at 10 Hz in sync
   - Predictable execution order
   - Easier to debug and monitor

4. **Proper ROS Design**
   - All functionality in one node as intended
   - Cleaner ROS topic namespace
   - Better follows ROS best practices

## Implementation Details

### Single Task Flow (10 Hz / 100ms period)

```cpp
void micro_ros_spin_task(void *arg)
{
    while (1) {
        // 1. Process incoming subscriptions
        rclc_executor_spin_some(&executor, ...);
        
        // 2. Read AS5600 sensors
        radians_motor2 = g_as5600_motor2->get_radians();
        rpm_motor2 = g_as5600_motor2->get_rpm();
        radians_motor3 = g_as5600_motor3->get_radians();
        rpm_motor3 = g_as5600_motor3->get_rpm();
        
        // 3. Publish sensor data
        rcl_publish(&sensor_publisher, &sensor_msg, NULL);
        
        // 4. Wait for next cycle (10 Hz)
        vTaskDelayUntil(&last_wake, 100ms);
    }
}
```

### Node Architecture

**Single Node:** `keyboard_motor_group2_group3_controller`

**Subscriptions:**
- `/keyboard/group2` (std_msgs/Int8) - Motor 2 control
- `/keyboard/group3` (std_msgs/Int8) - Motor 3 control

**Publications:**
- `/as5600_sensors` (std_msgs/Float32MultiArray) - Sensor data [rad2, rpm2, rad3, rpm3]

### Task List (Simplified)

1. **motor_control** (Priority 2) - Controls both motors based on commands
2. **micro_ros_spin** (Priority 1) - Handles all ROS communication (subscriptions + publishing)

### Mutex Usage

**Single Network Mutex:** `microros_network_mutex`
- Protects all WiFi/UDP operations
- Used by `rclc_executor_spin_some()` (receiving)
- Used by `rcl_publish()` (transmitting)
- Prevents lwIP buffer corruption

**Sensor Data Mutex:** `sensor_msg_mutex`
- Protects sensor message data structure
- Short-lived lock during data population
- Released before network operation

## Files Modified

### vaccum_firmware.cpp
- Removed `sensor_publish_task` creation
- Now only creates two tasks (was three)

### micro_ros_keyboard.cpp
- Added AS5600 sensor reading to `micro_ros_spin_task`
- Added sensor message allocation
- Integrated publishing into main loop
- Added external references to sensor objects

### as5600_sensor.cpp
- Removed `sensor_publish_task` function (no longer needed)
- Simplified to just initialization function
- Sensor objects now accessed directly from main task

### as5600_sensor.hpp
- Removed `sensor_publish_task` declaration
- Updated comments

## Performance

- **Task Count:** 2 tasks (was 3)
- **Memory Saved:** ~4KB stack from removed task
- **Update Rate:** Still 10 Hz for sensor publishing
- **Latency:** Actually improved - sensors read immediately after processing commands

## ROS Topic Structure

```
keyboard_motor_group2_group3_controller/
├── Subscriptions:
│   ├── /keyboard/group2 (std_msgs/Int8)
│   └── /keyboard/group3 (std_msgs/Int8)
└── Publishers:
    └── /as5600_sensors (std_msgs/Float32MultiArray)
```

## Testing

```bash
# View node info
ros2 node list
ros2 node info /keyboard_motor_group2_group3_controller

# Test subscriptions
ros2 topic pub /keyboard/group2 std_msgs/msg/Int8 "data: 1"
ros2 topic pub /keyboard/group3 std_msgs/msg/Int8 "data: -1"

# Monitor sensor publishing
ros2 topic echo /as5600_sensors
ros2 topic hz /as5600_sensors  # Should show ~10 Hz
```

## Conclusion

The consolidated architecture is cleaner, more efficient, and better follows ROS design principles by keeping all related functionality within a single node.

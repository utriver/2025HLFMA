# Recording Package

This ROS2 package provides a recording and playback system for control signals and angular targets.

## Features

- **Recording**: Subscribe to `/control_signal` and `/angular_target` topics at 100Hz and save data to CSV files
- **Playback**: Read CSV files and publish the recorded data back to the same topics at 100Hz
- **Keyboard Control**: Interactive control using keyboard keys

## Usage

### Building the Package

```bash
cd /home/yeoungjun/ros2_ws
colcon build --packages-select recoding
source install/setup.bash
```

### Running the Recording Node

```bash
ros2 run recoding recording
```

### Keyboard Controls

- **'r'**: Start/Stop recording
- **'p'**: Start/Stop playback
- **'q'**: Quit the application

### Recording Flow

1. Press 'r' to start recording
2. The node will subscribe to `/control_signal` and `/angular_target` topics
3. Data is recorded at 100Hz and temporarily stored in memory
4. Press 'r' again to stop recording and save to CSV file

### Playback Flow

1. Press 'p' to start playback
2. The node will find the most recent CSV file
3. Data is published to `/control_signal` and `/angular_target` topics at 100Hz
4. Press 'p' again to stop playback

## Topics

### Subscribed Topics
- `/control_signal` (std_msgs/Float64MultiArray): Control signal data
- `/angular_target` (geometry_msgs/Twist): Angular target data

### Published Topics
- `/control_signal` (std_msgs/Float64MultiArray): Control signal data (during playback)
- `/angular_target` (geometry_msgs/Twist): Angular target data (during playback)

## CSV File Format

The recorded CSV files contain the following columns:
- `timestamp`: Unix timestamp when data was recorded
- `control_data`: Control signal data array
- `angular_x`, `angular_y`, `angular_z`: Linear velocity components
- `angular_roll`, `angular_pitch`, `angular_yaw`: Angular velocity components

## Dependencies

- rclpy
- std_msgs
- geometry_msgs

# Offboard Control NED Node

## Overview
A ROS2 node for controlling PX4 drones using absolute positions in the NED (North-East-Down) coordinate frame, rather than relative positions in the local frame.

## Key Features
- Converts NED frame commands to local frame
- Supports multiple drones through namespacing
- Automatic takeoff and landing
- Position objective tracking
- Configurable initial heading

## Frame Conversion Details
### NED Frame (Input)
- X = North (+)
- Y = East (+)
- Z = Down (+)
- Origin: Fixed reference point
- Yaw: 0° = North, measured clockwise

### Local Frame (PX4)
- X = Forward
- Y = Right
- Z = Down
- Origin: Drone's initial position
- Yaw: 0° = Forward

### Conversion Process
1. NED coordinates are rotated by initial heading
2. No position offset needed (assuming drone starts at 0,0,0)
3. Yaw angles are adjusted by adding initial heading

## Usage

### Launch Command
```bash
ros2 run offboard_control_py offboard_control_ned_pose --ros-args -p initial_heading:=0.0 --remap __ns:=/px4_1
```

### Send Position Command
```bash
ros2 topic pub /px4_1/target_ned_pose geometry_msgs/msg/Pose "{
    position: {x: 10.0, y: 0.0, z: -5.0},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
}"
```

### Parameters
- `initial_heading`: Drone's heading relative to North (degrees)
- `TAKEOFF_HEIGHT`: Default 2.0m
- `POSITION_THRESHOLD`: 0.3m (distance to consider position reached)
- `POSE_TIMEOUT`: 20s (time without commands before auto-landing)

## State Machine
1. **Initialization**
   - Wait for FCU connection
   - Perform preflight checks
   - Capture initial position

2. **Takeoff**
   - Arm vehicle
   - Enter offboard mode
   - Rise to TAKEOFF_HEIGHT

3. **Navigation**
   - Receive NED pose commands
   - Transform to local frame
   - Navigate to position
   - Hold position when reached

4. **Landing**
   - Triggered by:
     - Mission completion
     - Command timeout
     - No new commands

## Important Notes
1. The drone must be positioned and oriented correctly before startup
2. NED frame commands are absolute positions
3. Z is positive downward (-2.0 means 2 meters up)
4. Yaw 0° is North, 90° is East, etc.

## Common Issues
1. **Incorrect Position Transformations**
   - Verify initial_heading parameter
   - Check coordinate frame conventions
   - Ensure proper yaw angle conversion

2. **Navigation Issues**
   - Check position threshold settings
   - Verify PX4 parameters
   - Monitor velocity and acceleration limits

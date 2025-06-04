# PX4 Multi-Drone Control System

A ROS 2 package providing high-level control interfaces for multiple PX4-powered drones, supporting different coordinate frames (NED, FRD, FLU).

## Features

- Multiple coordinate frame support:
  - NED (North-East-Down) for global navigation
  - FRD (Forward-Right-Down) for relative movement
  - FLU (Forward-Left-Up) for relative movement
- Multi-drone mission execution
- YAML-based mission configuration
- Automatic takeoff and landing
- Position holding with drift compensation
- Configurable hover times at waypoints
- Action Server and Topic interfaces
- Comprehensive error handling and retry logic
- Progress monitoring and feedback

## Prerequisites

- ROS 2 (Tested on Humble)
- PX4 Autopilot (SITL or hardware)
- px4_msgs package
- px4_controllers_interfaces package with:
  - PointYaw.msg
  - GotoPosition.action

## Installation

1. Clone this repository into your ROS 2 workspace:
```bash
git clone https://github.com/nbaudesson/SWARMz4.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select offboard_control_py px4_controllers_interfaces
```

3. Source the workspace:
```bash
source install/setup.bash
```

## System Architecture

### Launching the System

The system consists of multiple components that need to be launched:

1. **PX4 Offboard Controllers** - One per drone:
   ```bash
   ros2 launch offboard_control_py offboard_team.launch.py team_id:=1 coordinate_system:=NED
   ```

2. **Drone Behavior Controllers** - For customized behaviors:
   ```bash
   ros2 launch offboard_control_py offboard_clients.launch.py team_id:=1
   ```

3. **Single Drone Launch** - For testing:
   ```bash
   ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_1 -p coordinate_system:=NED
   ```

### Controller Types

1. **PX4 Unified Controller** (`offboard_control_px4`):
   - Low-level drone control
   - Supports multiple coordinate systems and control modes
   - Implements automatic takeoff, landing, and position holding

2. **Mission Controller** (`offboard_control_client`):
   - High-level behavior implementation
   - Uses a template structure for customizing drone behaviors
   - Communicates with `offboard_control_px4` via Action Client

## Control Interfaces

There are three main ways to control the drones:

### 1. Action Server Interface (Recommended)

The Action Server provides feedback and status tracking:

```bash
# Send goal with feedback
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition \
    "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}" --feedback

# Send goal and wait for result
ros2 action send_goal -w /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition \
    "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}"

# Cancel goal
ros2 action cancel /px4_1/goto_position
```

### 2. Topic Interface

Direct topic publishing for simpler control:

```bash
# Position command
ros2 topic pub /px4_1/target_pose px4_controllers_interfaces/msg/PointYaw \
    "{position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}"

# Velocity command (when in velocity mode)
ros2 topic pub /px4_1/target_pose px4_controllers_interfaces/msg/PointYaw \
    "{position: {x: 1.0, y: 0.0, z: 0.0}, yaw: 0.0}"
```

### 3. Programmatic Control

Using the provided template:

```python
# Example: Navigate to a position
def navigate_to_position(self):
    self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
    success = self.navigate_to(5.0, 0.0, -2.0, 0.0)
    if success:
        self.get_logger().info('Navigation command sent')

# Example: Send velocity commands
def move_forward(self):
    self.set_offboard_parameters(offboard_mode='velocity', coordinate_system='FLU')
    self.send_velocity(1.0, 0.0, 0.0, 0.0)  # Forward at 1 m/s
```

The `offboard_control_client_template.py` provides a comprehensive framework with:
- State machine for managing drone behavior
- Movement and navigation methods
- Combat functions
- Team communication
- Position tracking and utility functions

## Configuration Parameters

### Key Parameters for `offboard_control_px4`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `coordinate_system` | `'NED'` | Coordinate frame for commands |
| `offboard_mode` | `'position'` | Control mode (position/velocity) |
| `takeoff_height` | `5.0` | Height for takeoff in meters |
| `hover_timeout` | `10.0` | Time to hover before auto-landing |
| `position_threshold` | `0.15` | Distance threshold for position reached |
| `command_velocity_timeout` | `2.0` | Timeout for velocity commands |

### Coordinate Systems Explained

1. **NED Frame** (North-East-Down):
   - Global navigation frame
   - X: North (positive)
   - Y: East (positive)
   - Z: Down (positive, negative is up)
   - Yaw: 0° = North, 90° = East

2. **FRD Frame** (Forward-Right-Down):
   - Body-relative frame
   - X: Forward (aligned with drone's heading)
   - Y: Right (perpendicular to heading)
   - Z: Down (positive, negative is up)
   - Yaw: Relative to current heading

3. **FLU Frame** (Forward-Left-Up):
   - Body-relative frame
   - X: Forward (aligned with drone's heading)
   - Y: Left (perpendicular to heading)
   - Z: Up (positive, negative is down)
   - Yaw: Relative to current heading

### Control Modes

1. **Position Mode** (`offboard_mode: 'position'`):
   - Commands absolute positions in chosen coordinate frame
   - Drone calculates velocity to reach the target
   - More stable for precise positioning

2. **Velocity Mode** (`offboard_mode: 'velocity'`):
   - Direct control of vehicle velocity vector and yaw rate
   - Commands timeout after `command_velocity_timeout` seconds
   - Useful for smoother manual control or tracking

## Code Examples

### 1. Basic Position Control with NED Frame

```python
def mission_sequence(self):
    # Take off
    self.set_offboard_parameters('position', 'NED')
    self.navigate_to(0.0, 0.0, -5.0, 0.0)  # Up 5 meters
    
    # Fly to waypoint
    self.navigate_to(10.0, 0.0, -5.0, 90.0)  # 10m North, 90° heading
    
    # Return to base
    self.return_to_spawn(altitude=-3.0)
```

### 2. Relative Movement with FRD Frame

```python
def patrol_area(self):
    self.set_offboard_parameters('position', 'FRD')
    
    # Move 5m forward relative to current heading
    self.navigate_to(5.0, 0.0, 0.0, 0.0)
    
    # Turn 90° and move 3m right
    self.navigate_to(0.0, 3.0, 0.0, 90.0)
    
    # Move back 5m
    self.navigate_to(-5.0, 0.0, 0.0, 0.0)
```

### 3. Velocity Control Example

```python
def track_target(self, target_velocity):
    self.set_offboard_parameters('velocity', 'FLU')
    
    # Match target velocity
    self.send_velocity(
        target_velocity.x,
        target_velocity.y,
        target_velocity.z,
        target_velocity.yaw_rate
    )
```

## Advanced Usage

### State Machine Implementation

The template client implements a state machine with these states:
- `STATE_INIT`: Initial setup
- `STATE_TAKEOFF`: Executing takeoff sequence
- `STATE_MISSION`: Main mission execution
- `STATE_COMBAT`: Engaging targets
- `STATE_RTL`: Return to launch
- `STATE_COMPLETE`: Mission completion

Example state handler:
```python
def handle_mission_state(self):
    """Main mission behavior implementation"""
    if not self.action_in_progress:
        if not hasattr(self, 'waypoint_index'):
            self.waypoint_index = 0
            self.waypoints = [
                (10.0, 0.0, -5.0, 0.0),  # (x, y, z, yaw)
                (10.0, 10.0, -5.0, 90.0),
                (0.0, 10.0, -5.0, 180.0)
            ]
        
        # Get next waypoint
        if self.waypoint_index < len(self.waypoints):
            x, y, z, yaw = self.waypoints[self.waypoint_index]
            success = self.navigate_to(x, y, z, yaw)
            
            if success:
                self.waypoint_index += 1
        else:
            self.change_state(self.STATE_RTL)
```

### Multi-Drone Formation

Use the template to implement formation flying:

```python
class FormationController(DroneController):
    def __init__(self):
        super().__init__()
        self.formation_offset = (2.0, 2.0, 0.0)  # Relative to leader
    
    def follow_leader(self, leader_position):
        # Calculate formation position
        x = leader_position.x + self.formation_offset[0]
        y = leader_position.y + self.formation_offset[1]
        z = leader_position.z + self.formation_offset[2]
        
        # Navigate to formation position
        self.navigate_to(x, y, z, leader_position.yaw)
```

## Coordinate System Usage

### 1. NED Frame (Global Navigation)

Best for:
- Navigation between GPS waypoints
- Operations in known environments
- Coordinating multiple drones in absolute positions

```bash
# Launch with NED frame
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=NED
```

```python
# Go to absolute position
controller.navigate_to(10.0, 20.0, -5.0, 90.0)  # 10m North, 20m East, 5m altitude
```

### 2. FRD Frame (Relative Navigation)

Best for:
- Moving relative to current heading
- Obstacle avoidance maneuvers
- Visual-based navigation

```bash
# Launch with FRD frame
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=FRD
```

```python
# Move 5m forward, 2m right, stay at same height
controller.navigate_to(5.0, 2.0, 0.0, 0.0)
```

### 3. FLU Frame (Robotics Convention)

Best for:
- Compatibility with robotics algorithms
- Intuitive control (forward/left/up)
- Visual servoing applications

```bash
# Launch with FLU frame
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=FLU
```

```python
# Move 3m forward, 1m left, up 2m
controller.navigate_to(3.0, 1.0, 2.0, 0.0)
```
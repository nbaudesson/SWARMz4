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

## Controller Types

**PX4 Unified Controller** (`offboard_control_px4`):
   - Supports multiple coordinate systems
   - Configure with `coordinate_system` parameter
   - Available coordinate systems:
     - `NED`: Global position control (North-East-Down)
     - `FRD`: Relative position control (Forward-Right-Down)
     - `FLU`: Relative position control (Forward-Left-Up)
   - Position and velocity control modes

**Mission Control** (`offboard_control_client`):
   - Call several controllers through ROS2 action clients
   - Multi-drone mission execution
   - YAML mission configuration
   - Progress monitoring

## Coordinate Systems

1. **NED Frame**:
   - X: North (positive)
   - Y: East (positive)
   - Z: Down (positive)
   - Yaw: 0° = North, 90° = East

2. **FRD Frame**:
   - X: Forward (drone's heading)
   - Y: Right (relative to heading)
   - Z: Down (positive)
   - Yaw: Relative to current heading

3. **FLU Frame**:
   - X: Forward (drone's heading)
   - Y: Left (relative to heading)
   - Z: Up (positive)
   - Yaw: Relative to current heading

## Usage

### 1. Basic Position Control with NED Frame

Launch a single drone controller:
```bash
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=NED
```

Send position commands:
```bash
# Using action server (recommended)
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}}"

# Using topic interface
ros2 topic pub /px4_1/target_pose px4_controllers_interfaces/msg/PointYaw "{position: {x: 5.0, y: 0.0, z: -2.0}, yaw: 0.0}"
```

### 2. Relative Position Control with FRD Frame

Launch with FRD coordinate system:
```bash
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=FRD
```

Send relative movement commands:
```bash
# Move 5 meters forward
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: 0.0}, yaw: 0.0}}"

# Move 3 meters right and up 1 meter with 90° rotation
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 0.0, y: 3.0, z: -1.0}, yaw: 90.0}}"
```

### 3. Forward-Left-Up (FLU) Control

Launch with FLU coordinate system:
```bash
ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=FLU
```

Send commands in FLU frame:
```bash
# Move 5 meters forward
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 5.0, y: 0.0, z: 2.0}, yaw: 0.0}}"

# Move 3 meters left and up 1 meter with 90° rotation
ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition "{target: {position: {x: 0.0, y: 3.0, z: 1.0}, yaw: 90.0}}"
```

### 4. Velocity Control

The `offboard_control_px4` node also provides velocity control. Set the `offboard_mode` parameter to `velocity`:

```bash
# Launch with velocity control mode
ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_1 -p coordinate_system:=FLU -p offboard_mode:=velocity
```

Parameters for velocity control:
```yaml
max_horizontal_speed: 12.0     # Maximum horizontal velocity (m/s)
max_vertical_speed: 12.0       # Maximum vertical velocity (m/s)
max_yaw_rate: 10.0            # Maximum yaw rate (rad/s)
velocity_timeout: 2.0          # Time before zeroing velocity (s)
coordinate_system: 'FLU'      # Reference frame (FLU/FRD/NED)
takeoff_height: 2.0           # Target takeoff height (m)
hover_timeout: 10.0           # Hover time before auto-land (s)
land_height_threshold: 1.0    # Height to trigger landing (m)
```

Send velocity commands:
```bash
ros2 topic pub /px4_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

### 5. Multi-Drone Missions

1. Create a mission file (e.g., `missions/my_mission.yaml`):
```yaml
"1": [  # Drone 1 waypoints
  {x: 5.0, y: 0.0, z: -2.0, yaw: 90.0, time: 5.0},
  {x: 5.0, y: 5.0, z: -2.0, yaw: 180.0, time: 3.0}
]
"2": [  # Drone 2 waypoints
  {x: -5.0, y: 0.0, z: -2.0, yaw: -90.0, time: 5.0},
  {x: -5.0, y: -5.0, z: -2.0, yaw: -180.0, time: 3.0}
]
```

2. Launch the mission:
```bash
ros2 run offboard_control_py offboard_control_client --ros-args -p mission_file:=my_mission.yaml
```

## Configuration

### 1. Spawn Positions (`config/spawn_position.yaml`)
This file is automatically generated each time `launch_game.sh` is run. The script:
- Generates random spawn positions for each team within their designated field areas
- Team 1 spawns in the first 20% of field length
- Team 2 spawns in the last 20% of field length
- Drones within each team are arranged in a line with 2m spacing
- Transforms Gazebo coordinates to NED frame coordinates
- Updates the YAML file with the new positions

Example of generated spawn positions:
```yaml
"1": {  # Team 1
  "1": {x: 0.0, y: 0.0, yaw: 0.0},  # Drone 1
  "2": {x: 2.0, y: 0.0, yaw: 0.0}   # Drone 2
}
```

Manual editing of this file is not recommended as it will be overwritten on the next launch.

### 2. Controller Configuration (`config/controller_config.yaml`)
```yaml
"1": {  # Team 1
  "1": "NED",  # Drone 1 uses NED frame
  "2": "FRD"   # Drone 2 uses FRD frame
}
```

## Launch Files

1. `offboard_control.launch.py`: Basic controller launch to run several controllers for one team of drones
   ```bash
   ros2 launch offboard_control_py offboard_control.launch.py \
     team_id:=1 \
     coordinate_system:=NED \
     config_file:=my_config.yaml \
     spawn_file:=my_spawn.yaml
   ```

2. `game_test.launch.py` : Complete test environment : 2 teams launcher + Mission controller for both teams (not acceptable in actual games) + Game Master launcher (to run a game)
   ```bash
   ros2 launch offboard_control_py game_test.launch.py
   ```

## Advanced Usage

### Mission Control System
The `offboard_control_client` demonstrates how to control multiple drones using ROS2 action clients. It is an exemple code to show you how to multiple drones at once from a single node (in this case using the positions controllers). Here are some more exemples:

### Controller client Components

1. **Action Client Interface**
   - Each drone exposes a GotoPosition action server
   - Commands are sent as action goals
   - Feedback provides position and distance info
   - Results indicate success/failure

2. **Position Commands**
   ```python
   # Send absolute position (NED frame)
   ned_client.send_command(x=5.0, y=0.0, z=-2.0, yaw=0.0)
   
   # Send relative position (FRD frame)
   frd_client.send_command(x=2.0, y=1.0, z=0.0, yaw=90.0)
   ```

3. **Goal Tracking**
   ```python
   def goal_response_callback(self, future):
       goal_handle = future.result()
       if goal_handle.accepted:
           result_future = goal_handle.get_result_async()
           result_future.add_done_callback(self.get_result_callback)

   def get_result_callback(self, future):
       result = future.result().result
       if result.success:
           # Goal reached successfully
           self.handle_success()
       else:
           # Goal failed
           self.handle_failure()
   ```

### 3. Example: Simple Waypoint Controller

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from px4_controllers_interfaces.action import GotoPosition

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')
        
        # Create drone client
        self.drone = CustomDroneClient(self, drone_id='1')
        
        # Define waypoints
        self.waypoints = [
            {'x': 5.0, 'y': 0.0, 'z': -2.0, 'yaw': 0.0},
            {'x': 5.0, 'y': 5.0, 'z': -2.0, 'yaw': 90.0}
        ]
        self.current_waypoint = 0
        
        # Start mission
        self.send_next_waypoint()
        
    def send_next_waypoint(self):
        if self.current_waypoint < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint]
            self.drone.send_command(wp['x'], wp['y'], wp['z'], wp['yaw'])

    def handle_success(self):
        self.current_waypoint += 1
        self.send_next_waypoint()
```

### 4. Advanced Features

1. **Retry Logic**
   ```python
   class RetryableClient:
       def __init__(self, max_retries=3, retry_delay=2.0):
           self.max_retries = max_retries
           self.retry_delay = retry_delay
           self.current_retry = 0

       def handle_failure(self):
           if self.current_retry < self.max_retries:
               self.current_retry += 1
               self.node.create_timer(
                   self.retry_delay,
                   self.retry_waypoint
               )
   ```

2. **Progress Monitoring**
   ```python
   class MonitoredClient:
       def __init__(self):
           self.start_time = time.time()
           self.progress_interval = 5.0
           self.last_progress_time = 0

       def update_progress(self):
           current_time = time.time()
           if current_time - self.last_progress_time >= self.progress_interval:
               # Log progress
               self.last_progress_time = current_time
   ```

3. **Position Holding**
   ```python
   class HoldPositionClient:
       def hold_current_position(self):
           if self.current_pos:
               self.send_command(
                   self.current_pos.x,
                   self.current_pos.y,
                   self.current_pos.z,
                   self.current_yaw
               )
   ```

### 5. Best Practices

1. **Error Handling**
   - Always check for valid position data
   - Implement timeout protection
   - Handle connection losses gracefully

2. **Resource Management**
   - Clean up action clients properly
   - Cancel ongoing goals when needed
   - Release resources in destructors

3. **Safety Features**
   - Implement position validation
   - Add maximum velocity limits
   - Include emergency stop capability

4. **Testing**
   ```python
   def test_controller():
       rclpy.init()
       controller = WaypointController()
       try:
           rclpy.spin(controller)
       except KeyboardInterrupt:
           controller.destroy_node()
       rclpy.shutdown()
   ```

Remember that these are building blocks - you can mix and match features based on your specific needs. The key is to maintain robust error handling and clean resource management while implementing your custom control logic.
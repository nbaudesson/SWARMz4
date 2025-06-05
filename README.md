# SWARMz4: Drone and Ship Battle Challenge

## Description
SWARMz4 is a workspace for a drone and ship battle simulation environment in Gazebo. This repository provides a complete framework for multi-robot combat simulations, featuring autonomous drones and ships competing in teams. The system includes all necessary components for simulation, control, detection, communication, and game management.

## Features
- Gazebo simulation environment for drone and ship battles
- Integration with PX4, ROS2, and QGroundControl
- Custom ROS2 packages for game management
- Multi-team support with balanced team assignment
- Comprehensive detection and targeting systems
- Inter-robot communication within teams
- Missile and kamikaze attack systems
- Full simulation of physics, sensors, and combat

## System Architecture

### Data Communication
The following diagram shows the programs necessary to operate the platform and illustrates the data flow between components:

![SWARMz4 Architecture](images/swarmz4_architecture.jpg)
*Blue: ROS2 topics, Green: Services, Red: Actions*

### Launch Structure
This diagram shows how the launch files start all components for a full simulation:

![Launch Structure](images/launch_diagram.jpg)
*Each bubble represents a launch file or script. The red box highlights the code each team must implement.*

## Requirements
- Ubuntu 22.04 or higher
- ROS2 Humble
- Gazebo Garden
- PX4 Autopilot
- MAVROS
- Python 3.10+

## Installation
To install the project, follow these steps:

1. Clone the repository:
    ```bash
    git clone --recursive https://github.com/nbaudesson/SWARMz4.git
    ```

2. Navigate to the project directory:
    ```bash
    cd SWARMz4
    ```

3. Run the installation script:
    ```bash
    ./install_scripts/install_swarmz.sh
    ```
    Or, you can install each element manually with the command lines inside each component's script. In that case I recommend manually setting up the environment variable for your folder:
    ```bash
    export SWARMZ4_PATH="/path/to/SWARMz4"
    echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc
    ```

4. Build the ROS2 workspace:
    ```bash
    cd ros2_ws
    colcon build && source install/setup.bash
    ```

## Coordinate System Reference

Understanding the different coordinate systems is crucial for properly controlling drones in the simulation:

### 1. NED Frame (North-East-Down)
- **Standard aviation coordinate system**
- X-axis points to **North**
- Y-axis points to **East**
- Z-axis points **Down** (toward Earth)
- Yaw: 0° = North, 90° = East, 180° = South, 270° = West
- **Best for:** Global navigation between waypoints, coordinating multiple drones in absolute positions
- **Example:** `navigate_to(10.0, 20.0, -5.0, 90.0)` moves to 10m North, 20m East, 5m altitude, facing East

### 2. local_NED Frame
- **PX4's local reference frame**
- Origin is typically at the drone's spawn/takeoff position
- Same orientation as NED, but with a local origin
- X-axis points to **North** from origin
- Y-axis points to **East** from origin
- Z-axis points **Down** from origin
- **Best for:** Local navigation relative to takeoff point
- **Example:** `navigate_to(0.0, 0.0, -5.0, 0.0)` takes off to 5m above origin, facing North

### 3. FRD Frame (Forward-Right-Down)
- **Body-fixed frame relative to vehicle orientation**
- X-axis points to the **Front** of the drone
- Y-axis points to the **Right** of the drone
- Z-axis points **Down** from the drone
- Yaw is relative to current heading
- **Best for:** Moving relative to current heading, obstacle avoidance maneuvers
- **Example:** `navigate_to(5.0, 2.0, 0.0, 0.0)` moves 5m forward, 2m right, at same height

### 4. FLU Frame (Forward-Left-Up)
- **Body-fixed frame commonly used in robotics**
- X-axis points to the **Front** of the drone
- Y-axis points to the **Left** of the drone
- Z-axis points **Up** from the drone
- Yaw is relative to current heading
- **Best for:** Intuitive control, compatibility with robotics algorithms
- **Example:** `navigate_to(3.0, 1.0, 2.0, 0.0)` moves 3m forward, 1m left, 2m up

### Coordinate System Usage
```python
# Choose coordinate system when initializing controller
self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')

# Or set via ROS parameter when launching
ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_1 -p coordinate_system:=FRD
```

## Main Components

### 1. Game Master Node

The central controller for the entire simulation, managing game rules, teams, health tracking, detection, and communication.

**Key features:**
- Automatic team formation and balancing
- Health and damage management
- Simulates sensor detection ranges
- Coordinates inter-robot communication
- Manages missile firing and kamikaze attacks
- Tracks game state and declares winners

**Usage:**
```bash
ros2 launch game_master game_master.launch.py
```

### 2. Boat Control System

#### Boat Driver
Provides low-level control of ship movement and cannon targeting.

**Key features:**
- Controls ship thrusters
- Manages cannon movement through action server
- Handles ship physics in water environment

#### Boat Client
Template and example implementations for ship AI control.

**Key features:**
- Detection processing to identify targets
- Cannon aiming and firing systems
- Navigation algorithms for ship movement
- Communication with teammates

**Usage:**
```bash
ros2 run boat_driver boat_client_demo --ros-args -r __ns:=/flag_ship_1
```

### 3. Drone Control System

#### offboard_control_px4
Low-level drone control interface that communicates with PX4 flight controller.

**Key features:**
- Multiple coordinate system support (NED, FRD, FLU)
- Position and velocity control modes
- Automatic takeoff and landing sequences
- Action server for trajectory control

**Usage:**
```bash
ros2 run offboard_control_py offboard_control_px4 --ros-args -r __ns:=/px4_1 -p coordinate_system:=NED
```

#### offboard_control_client_template
Template for implementing drone AI and behavior.

**Key features:**
- State machine for mission phases
- Detection processing
- Combat actions (missiles, kamikaze)
- Team communication

#### offboard_control_client_demo
Example implementation of drone control for testing.

**Usage:**
```bash
ros2 run offboard_control_py offboard_control_client_demo --ros-args -r __ns:=/px4_1
```

## Drone Control Methods

The offboard control system provides three interfaces to control the drones:

### 1. Action Server Interface (Recommended)
Provides structured control with feedback and status tracking:

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
Use the provided template to implement control in your own code:

```python
# Navigate to position
def go_to_position(self):
    self.set_offboard_parameters(offboard_mode='position', coordinate_system='NED')
    success = self.navigate_to(10.0, 20.0, -5.0, 90.0)
    if success:
        self.get_logger().info('Moving to target position')

# Send velocity commands
def move_forward(self):
    self.set_offboard_parameters(offboard_mode='velocity', coordinate_system='FRD')
    self.send_velocity(1.0, 0.0, 0.0, 0.0)  # Forward at 1 m/s
```

## Usage

### Launching the Full Game

To launch a complete game simulation with all components:

```bash
ros2 launch game_master game_demo.launch.py
```

This will start:
- Gazebo simulation with the game environment
- Game master node
- All server nodes (drone controllers, cannon controllers)
- Team 1 and Team 2 client nodes

### Development Testing

For development and testing purposes, you can use:

```bash
ros2 launch game_master game_servers.launch.py  # Launch server nodes only
ros2 launch game_master game_master.launch.py   # Launch game master only
```

### Creating Custom Controllers

1. For drones: Copy and modify `offboard_control_client_template.py`
2. For ships: Copy and modify `boat_client_template.py`
3. Implement your AI and control algorithms
4. Test individual components before full integration

## Game Rules

- Two teams compete: each with 5 drones and 1 flagship ship
- Objective: Destroy the enemy flagship or have more health at the end of the time limit
- Drones have 1 health point, ships have 6 health points
- Drones can fire missiles (2 per drone) or self-destruct (kamikaze)
- Ships can fire missiles (4 per ship)
- Each robot can only detect others within specific ranges
- Communication range is limited between teammates
- Game ends when a flagship is destroyed or time (5 minutes) runs out

## Results and Replays

Game results are saved to:
- `~/SWARMz4/results/game_results.txt` (cumulative)
- `~/SWARMz4/results/individual_games/` (per-game)

You can replay a game using:
```bash
ros2 bag play ~/SWARMz4/results/bags/game_state_<timestamp>
```

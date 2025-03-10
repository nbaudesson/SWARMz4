# SWARMz4: Drone and Ship Battle Challenge

## Description
SWARMz4 is a workspace for a drone and ship battle challenge in a Gazebo simulation. This repository provides scripts to install the necessary tools and dependencies to run the simulation and control the drones.

## Features
- Gazebo simulation environment for drone and ship battles
- Integration with PX4, ROS2, and QGroundControl
- Custom ROS2 packages for game management
- Example packages for drone control

## Installation
To install the project, follow these steps:
1. Clone the repository:
    ```bash
    git clone https://github.com/nbaudesson/SWARMz4.git
    ```
2. Navigate to the project directory:
    ```bash
    cd SWARMz4
    ```
3. Run the installation script:
    ```bash
    ./install_scripts/install_swarmz.sh
    ```
4. Build the ROS2 workspace
    ```bash
    cd ros2_ws
    colcon build && source install/setup.bash
    ```

## Usage

### Launching Simulations

SWARMz4 provides two different launch scripts:

1. **Development Testing** (`launch_simulation.sh`):
   - Used for development and testing
   - Spawns drones in fixed, predictable positions
   - Simplified setup for controller development
   ```bash
   ./launch_scripts/launch_simulation.sh [HEADLESS] [NUM_DRONES_PER_TEAM] [FIELD_LENGTH] [FIELD_WIDTH] [WORLD]
   ```

2. **Game Environment** (`launch_game.sh`):
   - Official game launch script
   - Random team spawn positions within designated areas
   - Full game setup with proper team separation
   ```bash
   ./launch_scripts/launch_game.sh [HEADLESS] [SPAWN_FILE] [DRONES_PER_TEAM] [FIELD_LENGTH] [FIELD_WIDTH] [WORLD]
   ```

   Parameters:
   - `HEADLESS`: (0/1) GUI or headless mode (default: 1)
   - `SPAWN_FILE`: Path to save spawn positions (default: config/spawn_position.yaml)
   - `DRONES_PER_TEAM`: Number of drones per team (default: 5)
   - `FIELD_LENGTH`: Field length in meters (default: 500)
   - `FIELD_WIDTH`: Field width in meters (default: 250)
   - `WORLD`: Gazebo world file name (default: swarmz_world)

   Game Spawn Rules:
   - Team 1 spawns randomly in first 20% of field length
   - Team 2 spawns randomly in last 20% of field length
   - Drones within each team spawn in a line with 2m spacing
   - Team 1 faces forward (0°), Team 2 faces Team 1 (180°)
   - Spawn positions are recorded to YAML file for controller use

   Example:
   ```bash
   # Launch game with GUI and 3 drones per team
   ./launch_game.sh 0 default_spawn.yaml 3

   # Launch full game (5v5) in headless mode
   ./launch_game.sh 1 default_spawn.yaml 5 500 250 swarmz_world
   ```

   The script automatically:
   - Cleans up existing processes
   - Launches MicroXRCE-DDS Agent
   - Spawns PX4 instances for all drones
   - Starts QGroundControl
   - Launches Gazebo simulation
   - Sets up ROS2 bridges
   - Generates and saves spawn positions

### Drone Control Methods

1. **Direct Position Control**:
   ```bash
   # NED frame control (global coordinates)
   ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition \
     "{target: {position: {x: 10.0, y: 0.0, z: -2.0}, yaw: 0.0}}"

   # FRD frame control (relative coordinates)
   ros2 action send_goal /px4_1/goto_relative px4_controllers_interfaces/action/GotoPosition \
     "{target: {position: {x: 5.0, y: 0.0, z: 0.0}, yaw: 90.0}}"
   ```

2. **Velocity Control**:
   ```bash
   # Send velocity commands (FLU frame)
   ros2 topic pub /px4_1/cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
   ```

3. **Combat Actions**:
   ```bash
   # Fire missile
   ros2 service call /fire_missile swarmz_interfaces/srv/Missile \
     "{robot_name: '/px4_1', target_name: '/px4_6'}"

   # Trigger kamikaze
   ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze \
     "{robot_name: '/px4_1'}"
   ```

### Mission Configuration

1. **YAML Mission Format**:
   ```yaml
   "1":  # Team 1
     "1":  # Drone 1
       - {x: 10.0, y: 0.0, z: -2.0, yaw: 0.0, time: 5.0}
       - {x: 10.0, y: 10.0, z: -2.0, yaw: 90.0, time: 3.0}
     "2":  # Drone 2
       - {x: -10.0, y: 0.0, z: -2.0, yaw: 0.0, time: 5.0}
   ```

2. **Launch Mission**:
   ```bash
   ros2 run offboard_control_py offboard_control_client \
     --ros-args -p mission_file:=missions/my_mission.yaml
   ```

### Monitor and Debug

1. **View Robot Status**:
   ```bash
   # Health status
   ros2 topic echo /px4_1/health

   # Position
   ros2 topic echo /px4_1/local_position
   ```

2. **Game Status**:
   ```bash
   # Game time
   ros2 topic echo /game_master/time

   # Team scores
   ros2 topic echo /game_master/scores
   ```

3. **Logging**:
   - Game results: `~/SWARMz4/game_results/`
   - ROS2 logs: `~/.ros/log/`
   - PX4 logs: `~/SWARMz4/PX4-Autopilot/build/px4_sitl_default/logs/`

### Performance Tuning

1. **Controller Parameters** (`config/controller_config.yaml`):
   ```yaml
   max_velocity: 12.0          # Maximum velocity (m/s)
   position_tolerance: 0.3     # Position reaching tolerance (m)
   yaw_tolerance: 0.1         # Yaw alignment tolerance (rad)
   ```

2. **Game Balance** (orga only) (`config/game_config.yaml`):
   ```yaml
   missile_cooldown: 5.0      # Seconds between missile shots
   detection_range: 100.0     # Robot detection range (m)
   comm_range: 200.0         # Team communication range (m)
   ```

## Contact
For any questions or feedback, please contact [nicolas.baudesson@alten.com].

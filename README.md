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
    Or, you can install each element manually with the command lines inside each component's script. In that case I recommend that you manullay setup the environment variable to your folder.
    ```bash
    export SWAMRZ4_PATH="/path/to/SWARMz4"
    echo "export SWARMZ4_PATH=\"$SWARMZ4_PATH\"" >> ~/.bashrc
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
   Example:
   ```bash
   # Launch game with GUI and 2 drones per team close to each other on gazebo's default map
   ./launch_game.sh 0 2 5 2 default
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
   # Launch game with GUI
   ./launch_game.sh 0

   # Launch game (5v5) in headless mode
   ./launch_game.sh 1
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
   The PX4-autopilot allows you to command the drone and receive flight data using the [MAVlink protocol](https://docs.px4.io/main/en/middleware/mavlink.html).
   You are free to program the PX4 drones using any of the [PX4 API](https://docs.px4.io/main/en/robotics/).
   - [QGroundControl](https://docs.px4.io/main/en/getting_started/px4_basic_concepts.html#qgc)
   - [MAVlink SDK](https://docs.px4.io/main/en/robotics/mavsdk.html)
   - [ROS2](https://docs.px4.io/main/en/ros2/) 

   I have have added exemple offboard controllers in [offboard_control_py](https://github.com/nbaudesson/SWARMz4/tree/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/offboard_control_py) using the ROS2 API to control a drone using position command and velocity command. Or you can make your own offboard_control program for your drones. To do that you can use the [offboard_control.py] (https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/offboard_control_py/offboard_control.py) (or [.cpp](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_cpp/src/offboard_control.cpp)) as a template and set your inputs using [PX4's control modes](https://docs.px4.io/main/en/flight_modes/offboard.html)

1. **Position Control**:\
   The offboard [NED](https://docs.px4.io/main/en/contribute/notation.html) and [FRD](https://docs.px4.io/main/en/contribute/notation.html) controllers are [ROS2 action servers](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html), they can be run with :
   ```bash
   # Manually run offboard_control_ned for px4_1 specifying its spawn position to convert given objectives from global coordinates to local coordinates.
   ros2 run offboard_control_py offboard_control_ned --ros-args -r __ns:=/px4_1 -p spawn_x:=1.0 -p spawn_y:=0.0
   ```
   Or Use a [ros2 launcher](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) to run multiple controllers simultaneously using the [offboard_control.launch.py](https://github.com/nbaudesson/SWARMz4/tree/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/launch)
   ```bash
   # Launch NED type controllers for all the drones of team 1 using the spawn_position.yaml generated by launch_game.sh
   ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 controller_type:=NED
   ```
   You can then send objective request to the controller using a ros2 action client using the interface [GotoPosition.msg](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/px4_pkgs/px4_controllers_interfaces/action/GotoPosition.action)
   ```bash
   # NED frame control (global coordinates)
   ros2 action send_goal /px4_1/goto_position px4_controllers_interfaces/action/GotoPosition \
     "{target: {position: {x: 10.0, y: 0.0, z: -2.0}, yaw: 0.0}}"
   ```
   In NED (0,0,0) is the home of the gazebo simulation world (10,0,-2) is 2 meters in the air 10 meters to the north (from home).
   ```bash
   # FRD frame control (relative coordinates)
   # You need to launch an offboard_control_frd
   ros2 action send_goal /px4_2/goto_relative px4_controllers_interfaces/action/GotoPosition \
     "{target: {position: {x: 5.0, y: 0.0, z: 0.0}, yaw: 90.0}}"
   
   ```

2. **Velocity Control**:\
   To send classic ros2 [cmd_vel](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) to control the drone using velocity commands, you can use (or modify) [offboard_control_vel](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/offboard_control_py/offboard_control_vel.py)

   ```bash
   # Manually run offboard_control_vel for px4_1.
   ros2 run offboard_control_py offboard_control_vel --ros-args -r __ns:=/px4_1
   ```

   ```bash
   # Send velocity commands ([FLU]() frame)
   ros2 topic pub /px4_1/cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
   ```
   This will make the drone go 1m/s forward for 2 seconds (velocity_timeout).

3. **Combat Actions**:
   The tools to attack other robots are ros2 services in the game master package : 
   - the [missile_server](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/swarmz_pkgs/game_master/game_master/missile_server.py) that will apply damage to the closest robot aligned with the shooter (drone or canon).
   ```bash
   # Fire missile
   ros2 service call /fire_missile swarmz_interfaces/srv/Missile \
     "{robot_name: '/px4_1', target_name: '/px4_6'}"
   ```
   - the [kamikaze_server](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/swarmz_pkgs/game_master/game_master/kamikaze_server.py) that will apply damage to all robot within explosion range.
   ```bash
   # Trigger kamikaze
   ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze \
     "{robot_name: '/px4_1'}"
   ```
   To [call a ros2 service](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html) from a program you can you [game_master_client](https://github.com/nbaudesson/SWARMz4/tree/main/ros2_ws/src/swarmz_pkgs/game_master/exemple) programs as an exemple.

### Boat control
1. **Boat thruster controller** 
   For the control of boat's thrusters there are ROS2 topics available to publish directly the desired velocity, the topic have the following structure:
   `/model/flag_ship_{team_id}/joint/{thruster}_engine_propeller_joint/cmd_thrust`
   - **team_id**: you replace it with your team id (1 or 2)
   - **thruster**: you replace it with the thruster you wanna control (left or right)

   So if you wanna send a velocity command to the left thruster of team1's boat you can use the following command
   ```bash
   #Send velocity command to left thruster of team1 boat
   ros2 topic pub /model/flag_ship_1/joint/left_engine_propeller_joint/cmd_thrust std_msgs/msg/Float64 "data: 2.0" --once
   ```
   - **NOTE:** The velocity command must be within the limits [-2.5,2.5] 
   
   Moreover, if you have a game master node running you can recover the boat's position via topic `/flag_ship_{team_id}/pose`
   So if you run the following command you can get the current position and orientation of team1's boat
   ```bash
   #Get position and orientation of boat 1
   ros2 topic echo /flag_ship_1/pose
   ```

2. **Boat cannon controller** 
   For the cannon control, an action server in the boat_driver package is available to control the pitch and yaw of the cannon.

   The action request is defined with 3 parameters in the Cannon.action file :  
   - `pitch`: must be between [-1.57, 1.57]
   - `yaw`: must be between [-3.14, 3.14]
   - `target_ship` : name of the ship to control (example : flag_ship_1, flag_ship_2...)

   Start the action server in a terminal: 

   ```bash
   # Activate cannon controller server
   ros2 run boat_driver cannon_server
   ```

   You can then send a goal request to the controller with the following command:

   ```bash
   # Send a goal request to cannon controller server, you specify the target ship, yaw, and pitch desired
   ros2 action send_goal /cannon cannon_interfaces/action/Cannon "{'pitch': 1.0, 'yaw': -1.0, 'target_ship': 'flag_ship_1'}" --feedback
   ```

   When you send a goal using the action client in the terminal, you will receive feedback showing the current pitch and yaw orientations of the cannon, with an average speed of 0.3 rad/s.


### Mission Configuration

Once the offboard controllers are running, you can have your strategy nodes, or any other code producing new objectives for your drones to go to. You can call the offboard nodes with a client. Here is an exemple [offboard_control_client](https://github.com/nbaudesson/SWARMz4/blob/main/ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/offboard_control_py/offboard_control_client.py) where I the client node send succesivley objectives (could be NED, FRD depending on the controller launched) to the offboard server.

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
In this code I use the ros2 action interface, that allows me to see to have feedback on the drone accomplishment of its objective.

### Monitor and Debug
Here are some usefull topics that can give you on your drone's status.

1. **View Robot Status**:
   ```bash
   # Health status
   ros2 topic echo /px4_1/health

   # Position
   ros2 topic echo /px4_1/fmu/out/vehicule_local_position
   ```

2. **Game Status**:
   ```bash
   # Remaining game time
   ros2 topic echo /game_master/time

   # Team scores
   ros2 topic echo /game_master/scores
   ```

3. **Logging**:
   - Game results: `~/SWARMz4/game_results/`
   - ROS2 logs: `~/.ros/log/`
   - PX4 logs: `~/SWARMz4/PX4-Autopilot/build/px4_sitl_default/logs/`

### Game Master Demo

The Game Master Demo provides a comprehensive demonstration of the SWARMz4 simulation capabilities, including drone control, combat mechanics, and team coordination.

1. **What the Demo Demonstrates:**
   - Formation flying (drones arranging in a circle)
   - Drone-to-drone communication
   - Detection and tracking capabilities
   - Missile firing system
   - Kamikaze attack execution
   - Game timing and management

2. **Running the Demo:**
   ```bash
   # Launch the complete demo (game master, servers, drone controllers, and demo client)
   ros2 launch game_master game_master_demo.launch.py
   ```

3. **Demo Sequence:**
   - **Phase 1:** Drones take off and arrange in a circle formation
   - **Phase 2:** Circle formation completes
   - **Phase 3:** Message demonstration between drones
   - **Phase 4:** Detection system demonstration
   - **Phase 5:** Missile firing demonstration
   - **Phase 6:** Kamikaze attack demonstration
   - **Phase 7:** Game completion

4. **Technical Details:**
   - The demo automatically configures spawn positions for all drones
   - Launches necessary services (missile_server, kamikaze_server)
   - Configures all drones with offboard_control for position commands
   - Runs approximately 3 minutes with choreographed actions

5. **Learning Objectives:**
   - Understanding the Game Master system architecture
   - Observing drone-to-drone communication capabilities
   - Visualizing detection ranges and team awareness
   - Experiencing weapon systems and combat mechanics

The demo serves as both a validation of the simulation environment and a teaching tool for new users to understand the capabilities of the SWARMz4 platform.

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

## Documentation
#### PX4 : [ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html), [PX4 Github documentation for ROS2](https://github.com/PX4/PX4-Autopilot/tree/d72c2cc378cb6cb7b3a839fab05e4025123c5441/docs/en/ros2), [PX4 offboard user guide](https://px4.gitbook.io/px4-user-guide/flying/flight_modes/offboard)
#### ROS2 : [Official Humble tutorial](https://docs.ros.org/en/humble/Tutorials.html), [Humble youtube tutorial](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy)
#### Gazebo : [PX4 Multi-Vehicle Simulation with Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html), [PX4 Github documentation for gazebo](https://github.com/PX4/PX4-Autopilot/tree/d72c2cc378cb6cb7b3a839fab05e4025123c5441/docs/en/sim_gazebo_gz)

## Contact
For any questions or feedback, please contact [nicolas.baudesson@alten.com]().

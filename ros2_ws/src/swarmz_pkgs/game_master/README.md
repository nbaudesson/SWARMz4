# Game Master Package

## Overview
The Game Master package is the central controller for the multi-robot combat simulation game. It manages team formation, health tracking, robot detection, communication, scoring, and game state management. This document provides instructions on how to start a game, modify parameters, and interact with the game using the provided topics and services.

---

## Starting a Game

1. **Ensure Prerequisites**:
   - ROS2 and Gazebo are properly installed and configured.
   - Use the `launch_game.sh` script located in the `launch_scripts` folder to launch the Gazebo simulation with the appropriate world file. Run the following command:
     ```bash
     ./launch_scripts/launch_game.sh
     ```

2. **Launch the Game Master Node**:
   Use the following command to start the Game Master node:
   ```bash
   ros2 launch game_master game_master.launch.py
   ```

3. **Monitor Logs**:
   The Game Master node will log information about detected robots, team formation, and game state.

---

## Parameters

The Game Master node uses parameters defined in the `game_master_params.yaml` file. These parameters control various aspects of the game, such as detection ranges, health, and missile system settings.

### Key Parameters

- **Detection Ranges**:
  - `drone_detection_range` (default: 137.0): `=height*55% for 2 horizontal corridors` Maximum range at which drones can detect others.
  - `ship2ship_detection_range` (default: 500.0): Maximum range at which ships can detect others. Can see the other ship across the map.
  - `ship2drone_detection_range` (default: 162.0): `=height*65% Can smaller objects only closer, but has an edge on the drones` Maximum range at which ships can detect drones.

- **Communication Ranges**:
  - `drone_communication_range` (default: 144.0): `=drone_detection_range*105%` Maximum range for drone communications.
  - `ship_communication_range` (default: 525.0): `=ship2ship_detection_range*105%` Maximum range for ship communications.

- **Health**:
  - `drone_health` (default: 1): Initial health points for drones. Instant kill for drones.
  - `ship_health` (default: 6): Initial health points for ships. Increased ship health to make fleet battles more interesting

- **Missile System**:
  - `drone_missile_range` (default: 69.0): `=drone_detection_range*50%` Maximum range for drone missiles.
  - `ship_missile_range` (default: 81.0): `=ship2drone_detection_range*50%` Maximum range for ship missiles.
  - `drone_missile_damage` (default: 1): Damage dealt by drone missiles.
  - `ship_missile_damage` (default: 1): Damage dealt by ship missiles.
  - `drone_cooldown` (default: 8.0): Cooldown time between drone shots.
  - `ship_cooldown` (default: 6.0): Cooldown time between ship shots.
  - `drone_magazine` (default: 2): Number of missiles per drone.
  - `ship_magazine` (default: 4): Number of missiles per ship.

- **Kamikaze System**:
  - `explosion_damage` (default: 3): Amount of damage dealt by kamikaze explosions.
  - `explosion_range` (default: 5.0): Radius of kamikaze explosion effect.

- **Game Settings**:
  - `game_duration` (default: 300): Duration of the game in seconds. Comment: `5 minutes`
  - `gazebo_world_name` (default: "game_world_water"): Name of the Gazebo world.

### Modifying Parameters

To modify parameters, edit the `game_master_params.yaml` file located in the `config` directory of the `game_master` package. After making changes, restart the Game Master node to apply the new settings.

---

## Interacting with the Game

### Topics

#### Game Master Topics
- **`/game_master/*`**:
  - These topics are for the Game Master's internal use only.
  - Gamers should not subscribe to these topics.

#### Robot-Specific Topics
- **`<robot_name>/health`**:
  - Publishes the health status of the robot.
  - **Message Type**: `std_msgs/msg/Int32`
  - Only the robot with the corresponding name should subscribe to this topic.

- **`<robot_name>/detections`**:
  - Publishes detection information for the robot.
  - **Message Type**: `swarmz_interfaces/msg/Detections`
  - Only the robot with the corresponding name should subscribe to this topic.

- **`<robot_name>/incoming_messages`**:
  - Publishes messages received by the robot from other robots.
  - **Message Type**: `std_msgs/msg/String`
  - Only the robot with the corresponding name should subscribe to this topic.

- **`<robot_name>/out_going_messages`**:
  - Allows the robot to publish messages intended for other robots.
  - **Message Type**: `std_msgs/msg/String`

### Communication Rules

- Each robot must be controlled by an independent ROS2 program.
- A robot can only subscribe to its own topics.
- Robots can communicate with each other using the `incoming_messages` and `out_going_messages` topics, which are managed by the Game Master.
- A user program controlling a robot is not allowed to look into the topics of other robots, even if they are on the same team.

### Services

#### Missile Firing
- **Service Name**: `/fire_missile`
- **Type**: `swarmz_interfaces/srv/Missile`
- **Request Format**:
  ```yaml
  robot_name: string  # Name of the robot firing the missile
  ```
- **Response Format**:
  ```yaml
  has_fired: bool  # Indicates if the missile was successfully fired
  ammo: int32      # Remaining ammunition for the robot
  ```
- **Description**: This service is managed by the Game Master node and provides a single interface for all robots to fire missiles. Robots are not allowed to request this service in the name of other robots. The `robot_name` in the request must match the name of the controlled robot.
- **Usage**:
  ```bash
  ros2 service call /fire_missile swarmz_interfaces/srv/Missile "{robot_name: '<robot_name>'}"
  ```
  Replace `<robot_name>` with the namespace of the robot firing the missile.

#### Kamikaze
- **Service Name**: `/kamikaze`
- **Type**: `swarmz_interfaces/srv/Kamikaze`
- **Request Format**:
  ```yaml
  robot_name: string  # Name of the robot initiating the kamikaze attack
  ```
- **Response Format**:
  ```yaml
  # Empty response
  ```
- **Description**: This service is managed by the Game Master node and provides a single interface for all robots to execute kamikaze actions. Robots are not allowed to request this service in the name of other robots. The `robot_name` in the request must match the name of the controlled robot.
- **Usage**:
  ```bash
  ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: '<robot_name>'}"
  ```
  Replace `<robot_name>` with the namespace of the robot initiating the kamikaze action.

#### Examples
For examples on how to use these services, refer to the `game_master_client_dynamic_test.py` file.

---

## Example Code and Launch File

The `game_master_demo.launch.py` file is a convenience launch file that runs the `game_master_node` along with the `game_master_client_dynamic_test.py` example client. This setup demonstrates the features of the Game Master system in a choreographed simulation.

### Features Demonstrated
- Formation flying (arranging drones in a circle)
- Inter-drone messaging
- Detection visualization
- Missile firing
- Kamikaze attacks
- Game timing and completion

### Demo Sequence
1. Drones take off and arrange in a circle at the center of the field (~1 minute).
2. Drone `/px4_1` sends a test message to demonstrate communication.
3. Detection outputs are displayed from `/px4_1` and `/px4_6`.
4. Drones `/px4_2` and `/px4_9` fire missiles.
5. Drone `/px4_3` executes a kamikaze attack.
6. Demo continues until game timeout.

### How to Run
- Use the following command to launch the demo:
  ```bash
  ros2 launch game_master game_master_demo.launch.py
  ```
  This will start the Game Master node, missile server, kamikaze server, and the example client in one terminal.

- Alternatively, you can run the example client separately after starting the Game Master node:
  ```bash
  ros2 run game_master game_master_client_dynamic_test.py
  ```

---

## Results

- Game results are saved in the `results` directory under the `SWARMz4` folder.
- Individual game results are stored in timestamped files within the `individual_games` subdirectory.

---

For further details, refer to the source code and comments in the `game_master` package.
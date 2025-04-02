# Game Master Package

A ROS2 package that manages combat simulation between multiple drones and ships in Gazebo. This package provides game mechanics including health management, damage calculation, missile firing, and kamikaze actions.

## Features

- Multi-team combat simulation management
- Health and damage tracking for each robot
- Missile firing system with configurable parameters
- Kamikaze (self-destruct) action system
- Automatic team formation
- Real-time detection and communication range simulation
- Score tracking and game results logging

## Prerequisites

- ROS2 (Tested on Humble)
- Gazebo
- PX4 Autopilot (for drone control)
- Python 3.8+

Required ROS2 packages:
```bash
ros2_ws/src/
├── px4_pkgs/
│   └── px4_controllers/
└── swarmz_pkgs/
    └── swarmz_interfaces/  # Custom message/service definitions
```

## Installation

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src/swarmz_pkgs
git clone https://github.com/yourusername/game_master.git
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select game_master swarmz_interfaces
```

3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Configuration

### Parameters

Game Master Node:
- `drone_detection_range` (float, default: 137.0): Maximum detection range for drones (height*55% for 2 horizontal corridors)
- `ship2ship_detection_range` (float, default: 500.0): Maximum detection range between ships (can see across the map)
- `ship2drone_detection_range` (float, default: 162.0): Maximum range for ships to detect drones (for smaller objects)
- `drone_communication_range` (float, default: 144.0): Maximum communication range for drones (drone_detection_range*105%)
- `ship_communication_range` (float, default: 30.0): Maximum communication range for ships (ship2ship_detection_range*105%)
- `drone_health` (int, default: 1): Initial health points for drones
- `ship_health` (int, default: 6): Initial health points for ships
- `drone_points` (int, default: 10): Points awarded for destroying a drone
- `ship_points` (int, default: 50): Points awarded for destroying a ship
- `game_duration` (int, default: 200): Game duration in seconds
- `world_name` (string, default: "swarmz_world_2"): Name of the simulation world
- `drone_max_speed` (float, default: 12.0): Maximum speed for drones

Missile Server:
- `drone_missile_range` (float, default: 69.0): Maximum missile range for drones (drone_detection_range*50%)
- `ship_missile_range` (float, default: 81.0): Maximum missile range for ships (ship2drone_detection_range*50%)
- `drone_missile_damage` (int, default: 1): Missile damage for drones
- `ship_missile_damage` (int, default: 1): Missile damage for ships
- `drone_cooldown` (float, default: 8.0): Cooldown time between drone missile fires
- `ship_cooldown` (float, default: 6.0): Cooldown time between ship missile fires
- `drone_magazine` (int, default: 2): Number of missiles a drone can fire before reload
- `ship_magazine` (int, default: 4): Number of missiles a ship can fire before reload
- `laser_width` (float, default: 3.0): Width of the laser beam for targeting

Kamikaze Server:
- `explosion_damage` (int, default: 3): Damage dealt by kamikaze explosion
- `explosion_range` (float, default: 5.0): Radius of explosion effect

## Game Rules

### Overview
The game is a team-based combat simulation between multiple drones and ships in Gazebo. The objective is to eliminate enemy robots and score points.

### Teams and Scoring
- Robots are automatically assigned to teams at the start of the game
- Destroying an enemy drone awards 10 points to your team
- Destroying an enemy ship awards 50 points to your team
- The team with the highest score at the end of the game duration (200 seconds) wins

### Health and Damage
- Drones start with 1 health point
- Ships start with 6 health points
- When health reaches 0, the robot is considered destroyed
- Drones deal 1 damage with missiles
- Ships deal 1 damage with missiles
- Kamikaze explosions deal 3 damage to all enemies within range

### Detection and Communication
- Drones can detect other robots within 137 units
- Ships can detect other ships across the entire map (500 units)
- Ships can detect drones within 162 units
- Drones can communicate with teammates within 144 units
- Ships can communicate with teammates within 30 units

### Weapons and Cooldowns
- Drones can fire missiles with a range of 69 units
- Ships can fire missiles with a range of 81 units
- Drones have a cooldown of 8 seconds between missile fires
- Ships have a cooldown of 6 seconds between missile fires
- Drones can fire 2 missiles before needing to reload
- Ships can fire 4 missiles before needing to reload
- All robots can perform a kamikaze action as a last resort

## Usage

1. Launch the simulation environment (PX4 SITL and Gazebo):
```bash
ros2 launch px4_controllers simulation.launch.py
```

2. Launch the game master system:
```bash
ros2 launch game_master game_master.launch.py
```

### Service Calls

Test missile firing:
```bash
ros2 service call /fire_missile swarmz_interfaces/srv/Missile "{robot_name: '/px4_1'}"
```

Trigger kamikaze action:
```bash
ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: '/px4_1'}"
```

## Node Architecture

### game_master_node

Central node managing game state, health, and scoring:
- Maintains robot health and team scores
- Processes damage events
- Manages game duration and completion
- Logs game results

### missile_server

Handles missile firing mechanics:
- Validates firing requests
- Checks target alignment and range
- Applies damage to targets
- Manages ammunition and cooldown

### kamikaze_server

Manages self-destruct actions:
- Processes kamikaze requests
- Calculates explosion effects
- Applies damage to nearby robots

## Topics and Services

### Published Topics
- `/<robot_name>/health` (std_msgs/Int32): Current robot health
- `/<robot_name>/detections` (swarmz_interfaces/Detections): Detected robots
- `/game_master/time` (std_msgs/Int32): Remaining game time

### Services
- `/fire_missile` (swarmz_interfaces/Missile): Request missile firing
- `/kamikaze` (swarmz_interfaces/Kamikaze): Trigger kamikaze action
- `/update_health` (swarmz_interfaces/UpdateHealth): Update robot health

## Game Results

Game results are stored in two locations:
- `~/SWARMz4/game_results/game_results.txt`: Cumulative results
- `~/SWARMz4/game_results/individual_games/`: Individual game results

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

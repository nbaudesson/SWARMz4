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
- `drone_detection_range` (float, default: 10.0): Maximum detection range for drones
- `ship_detection_range` (float, default: 20.0): Maximum detection range for ships
- `drone_health` (int, default: 100): Initial health points for drones
- `ship_health` (int, default: 200): Initial health points for ships
- `game_duration` (int, default: 300): Game duration in seconds

Missile Server:
- `drone_missile_range` (float, default: 100.0): Maximum missile range for drones
- `ship_missile_range` (float, default: 200.0): Maximum missile range for ships
- `drone_missile_damage` (int, default: 10): Missile damage for drones
- `ship_missile_damage` (int, default: 20): Missile damage for ships

Kamikaze Server:
- `explosion_damage` (int, default: 100): Damage dealt by kamikaze explosion
- `explosion_range` (float, default: 6.0): Radius of explosion effect

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

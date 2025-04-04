# Game Master Package

A ROS2 package that manages multi-robot combat simulation in Gazebo. This system orchestrates battles between teams of drones and warships, handling detection, health tracking, damage calculation, and game state management.

## Game Overview

### Basic Concept
Two teams compete in a simulated battlefield using PX4-controlled drones and flagship warships. The objective is to eliminate the enemy flagship or achieve air superiority by destroying enemy drones.

### Teams and Units
- Teams are automatically formed with equal numbers when possible
- Drones (PX4-controlled quadcopters) are small, agile units with limited health
- Flagships are larger vessels with more health points and different detection capabilities

### Game Mechanics
- **Health System**: Drones have 1 health point (instant kill), flagships have 6 health points
- **Detection**: Units can only target enemies within their detection range
- **Communication**: Units can only share information with teammates within communication range
- **Weapons**: Units can fire missiles that deal damage when they hit enemies
- **Kamikaze**: Units can self-destruct to damage multiple nearby enemies

### Win Conditions (in priority order)
1. Destroy the enemy flagship
2. Have the flagship with more remaining health
3. Have more surviving drones than the enemy team
4. Draw if all conditions are equal

## Node Architecture

The system consists of three main nodes that work together:

### 1. Game Master Node

**Purpose**: Central manager for game state, health tracking, and win condition evaluation.

**How it works**:
- Detects all participating robots at startup and forms teams automatically
- Tracks health points for each unit
- Simulates detection and communication ranges for all units
- Monitors game time and provides countdown
- Handles game termination and results logging
- Removes destroyed units from simulation

**Inputs**:
- Health update requests (via `/update_health` service)
- Robot poses (via Gazebo subscription)

**Outputs**:
- Health status (via `/<robot_name>/health` topics)
- Detection data (via `/<robot_name>/detections` topics)
- Communication forwarding (via `/<robot_name>/out_going_messages` topics)
- Game time (via `/game_master/time` topic)
- Game results (written to files)

### 2. Missile Server

**Purpose**: Handles missile firing mechanics and hit detection.

**How it works**:
- Processes missile firing requests from robots
- Validates if the shooter has ammunition and isn't in cooldown
- Checks if any valid targets are aligned with the shooter's orientation
- Applies damage to the closest aligned target
- Manages ammunition count and reload timers

**Inputs**:
- Missile firing requests (via `/fire_missile` service)
- Robot poses (via Gazebo subscription)
- Health updates (via subscription to health topics)

**Outputs**:
- Damage requests (via calls to `/update_health` service)
- Firing confirmation and ammo count (via service response)

### 3. Kamikaze Server

**Purpose**: Manages self-destruct actions and area damage.

**How it works**:
- Processes kamikaze (self-destruct) requests from robots
- Identifies all robots within explosion range
- Applies damage to all affected robots including the kamikaze unit
- Handles health updates for all affected units

**Inputs**:
- Kamikaze requests (via `/kamikaze` service)
- Robot poses (via Gazebo subscription)

**Outputs**:
- Damage requests (via calls to `/update_health` service)

## Detection and Targeting System

The detection system simulates sensors with specific ranges:

- Drones can detect other units within 137 units
- Ships can detect other ships from 500 units (across the map)
- Ships can detect drones from 162 units
- Detection information includes:
  - Unit type (drone/ship)
  - Friendship status (friend/foe)
  - Relative position (in FRD coordinates)

## Communication System

The communication system allows units to share information:

- Drones can communicate with teammates within 144 units
- Ships can communicate with teammates within 30 units
- Messages from one unit are forwarded to all friendly units within range

## Weapon Systems

### Missiles
- Drones: Range 69 units, 1 damage, 8s cooldown, 2 shots before reload
- Ships: Range 81 units, 1 damage, 6s cooldown, 4 shots before reload

### Kamikaze (Self-Destruct)
- 3 damage to all units within 5 units
- Destroys the activating unit

## Configuration Parameters

### Game Master Node
```yaml
game_master_node:
  ros__parameters:
    drone_detection_range: 137.0      # Maximum detection range for drones
    ship2ship_detection_range: 500.0  # Ship-to-ship detection range
    ship2drone_detection_range: 162.0 # Ship-to-drone detection range
    drone_communication_range: 144.0  # Drone communication range
    ship_communication_range: 30.0    # Ship communication range
    drone_health: 1                   # Drone initial health points
    ship_health: 6                    # Ship initial health points
    game_duration: 20                 # Game duration in seconds
    gazebo_world_name: "swarmz_world_2"     # Gazebo world name
    drone_model_base_name: "x500_lidar_front"  # Drone model base name
```

### Missile Server
```yaml
missile_server:
  ros__parameters:
    drone_missile_range: 69.0   # Drone missile range
    ship_missile_range: 81.0    # Ship missile range
    drone_missile_damage: 1     # Damage dealt by drone missiles
    ship_missile_damage: 1      # Damage dealt by ship missiles
    drone_cooldown: 8.0         # Cooldown between drone shots
    ship_cooldown: 6.0          # Cooldown between ship shots
    drone_magazine: 2           # Missiles per drone
    ship_magazine: 4            # Missiles per ship
    laser_width: 3.0            # Targeting beam width
    drone_padding_x/y/z: 0.5    # Drone hitbox size
    ship_padding_x: 6.0         # Ship hitbox X dimension
    ship_padding_y: 1.0         # Ship hitbox Y dimension
    ship_padding_z: 1.0         # Ship hitbox Z dimension
```

### Kamikaze Server
```yaml
kamikaze_server:
  ros__parameters:
    explosion_damage: 3        # Damage dealt by explosion
    explosion_range: 5.0       # Radius of explosion effect
```

## Usage Examples

### Launching the Game
```bash
/home/nb_adm/SWARMz4/launch_scripts/launch_game.sh [HEADLESS_LEVEL] [SPAWN_FILE] [DRONES_PER_TEAM]
```

Example: Launch with 5 drones per team in GUI mode:
```bash
/home/nb_adm/SWARMz4/launch_scripts/launch_game.sh 0 default 5
```

### Manual Service Calls

Fire a missile from drone 1:
```bash
ros2 service call /fire_missile swarmz_interfaces/srv/Missile "{robot_name: '/px4_1'}"
```

Trigger kamikaze action for drone 2:
```bash
ros2 service call /kamikaze swarmz_interfaces/srv/Kamikaze "{robot_name: '/px4_2'}"
```

Update health manually:
```bash
ros2 service call /update_health swarmz_interfaces/srv/UpdateHealth "{robot_name: '/px4_3', damage: 1}"
```

## Game Results

Game results are stored in:
- Cumulative results: `~/SWARMz4/results/game_results.txt`
- Individual game results: `~/SWARMz4/results/individual_games/game_results_TIMESTAMP.txt`

Example result:
```
--- Game 1 ---
Game Over
Team 1 wins! (Has more surviving drones)
Team 1 flagship health: 6/6
Team 1 drones: 5 surviving, 0 destroyed
Team 2 flagship health: 6/6
Team 2 drones: 3 surviving, 2 destroyed
```

## Known Issues

- Game number counting in result files may be incorrect when multiple games are run in sequence
- When running without flagships, the game will only use drone counts to determine winners
- If all drones on both teams are destroyed simultaneously, the game may declare a draw instead of evaluating flagship health

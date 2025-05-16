# SWARMz4 Drone Combat Simulation Demo

This demonstration showcases a coordinated team of 5 drones executing different tactical roles in a simulated combat environment.

## Overview

The demo presents a team of drones (Team 1) working together in a tactical scenario. Each drone has a specific role and behavior pattern:

- **Drone 0**: Reconnaissance and attack
- **Drone 1**: Communications relay
- **Drone 2**: Kamikaze attacker 
- **Drone 3**: Ship escort (front)
- **Drone 4**: Formation support

These drones interact with a simulated environment managed by the Game Master, which tracks positions, detections, health, and combat actions.

## Drone Team Behaviors

### Drone 0: Reconnaissance & Attack
- Takes off to 10m altitude
- Navigates to position (x=130, y=240)
- Scans for enemy drones approximately 25m in front
- When enemy detected:
  - Broadcasts enemy position and own coordinates
  - Waits 1 second
  - Fires missile at enemy

### Drone 1: Communications Support
- Takes off to 10m altitude
- Positions at (x=125, y=240), near Drone 0
- Listens for and displays messages from Drone 0
- Acts as communications relay

### Drone 2: Kamikaze Drone
- Takes off to 3m altitude
- Locates friendly ship
- Positions 2m above ship, following its movement
- After 15 seconds, executes kamikaze attack

### Drone 3: Ship Escort (Front)
- Takes off to 1m altitude
- Locates friendly ship
- Maintains position 10m in front of ship at 1m altitude
- Continuously adjusts position as ship moves

### Drone 4: Formation Support
- Takes off to 1m altitude
- Locates Drone 3
- Maintains position 5m south of Drone 3
- Follows Drone 3 in formation

## Running the Demo

The demo consists of three main components that need to be launched in the correct order:

### 1. Start PX4 SITL Simulation

First, start the PX4 SITL simulation environment with multiple drone instances:

```bash
cd ~/SWARMz4
./launch_game.sh
```

Wait until all drones are properly initialized in the Gazebo environment (usually takes 10-15 seconds).

### 2. Launch the Game Master

Next, launch the Game Master node that manages the simulation environment:

```bash
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 launch game_master game_master.launch.py
```

### 3. Launch Drone Controllers

Launch the drone controllers for Team 1:

```bash
# In a new terminal window
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 launch offboard_control_py offboard_team.launch.py team_id:=1
```

### 4. Launch All Drone Behaviors

Launch all drone behaviors simultaneously using the offboard_clients.launch.py file:

```bash
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 launch offboard_control_py offboard_clients.launch.py team_id:=1
```

This launches all five drone behavior controllers at once, each implementing its specific role in the tactical scenario.

### Alternative: Launch Individual Drone Behaviors Manually

If you prefer more control or need to debug specific drone behaviors, you can launch each drone's controller individually in separate terminal windows:

```bash
# Terminal 1 - Drone 0 (Reconnaissance)
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 run offboard_control_py offboard_control_client_0 --ros-args -r __ns:=/px4_0

# Terminal 2 - Drone 1 (Communications)
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 run offboard_control_py offboard_control_client_1 --ros-args -r __ns:=/px4_1

# Terminal 3 - Drone 2 (Kamikaze)
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 run offboard_control_py offboard_control_client_2 --ros-args -r __ns:=/px4_2

# Terminal 4 - Drone 3 (Ship Escort)
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 run offboard_control_py offboard_control_client_3 --ros-args -r __ns:=/px4_3

# Terminal 5 - Drone 4 (Formation Support)
source ~/SWARMz4/ros2_ws/install/setup.bash
ros2 run offboard_control_py offboard_control_client_4 --ros-args -r __ns:=/px4_4
```

Note: This manual approach is more cumbersome but can be useful for troubleshooting specific drones.

## Monitoring the Demo

### View Drone Status
```bash
ros2 topic echo /px4_0/health  # Replace with appropriate drone ID
```

### Monitor Detections
```bash
ros2 topic echo /px4_0/detections  # Replace with appropriate drone ID
```

### View Messages Between Drones
```bash
ros2 topic echo /px4_0/out_going_messages  # Messages sent by drone 0
ros2 topic echo /px4_1/out_going_messages  # Messages sent by drone 1
```

### Visualize in Gazebo
Watch the Gazebo simulation window to see the physical movement of drones and other entities.

## Demo Sequence

1. All drones take off to their respective altitudes
2. Drones 2, 3, and 4 locate and position themselves relative to the friendly ship
3. Drones 0 and 1 navigate to their assigned positions
4. Drone 0 scans for enemy drones and sends alerts to Drone 1
5. Drone 0 engages enemy drones with missiles
6. Drone 2 executes kamikaze attack after 15 seconds
7. Drones 3 and 4 continue escorting the friendly ship in formation

## Troubleshooting

- **Issue**: PX4 SITL instances fail to start
  **Solution**: Check if there are existing instances and kill them
  ```bash
  killall px4 gazebo gzserver gzclient
  ```

- **Issue**: Drones not detecting entities
  **Solution**: Ensure the game master is running and publishing detection messages
  ```bash
  ros2 topic list | grep detection
  ```

- **Issue**: Missile firing fails
  **Solution**: Check that the game master missile service is available
  ```bash
  ros2 service list | grep missile
  ```

- **Issue**: Drones not moving to positions
  **Solution**: Verify the action servers are running
  ```bash
  ros2 action list
  ```

## Shutting Down

To properly shut down the demo:

1. Terminate all drone controller nodes with Ctrl+C
2. Stop the Game Master node with Ctrl+C
3. Stop the PX4 SITL simulation:
```bash
cd ~/SWARMz4
./shutdown_swarmz.py
```
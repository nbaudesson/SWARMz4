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
To run a game, you need to start the Gazebo simulation with the appropriate number of robots. The SWARMz4 challenge makes two teams of 5 drones and 1 flagship fight each other over a 500 x 250 m field.

### Starting the Simulation
1. Run the launch script:
    ```bash
    ./launch_scripts/launch_simulation.sh [HEADLESS] [NUM_DRONES_PER_TEAM] [FIELD_LENGTH] [FIELD_WIDTH] [WORLD]
    ```
    - `HEADLESS`: Set to `1` for headless mode (default), `0` for GUI mode.
    - `NUM_DRONES_PER_TEAM`: Number of drones per team (default is 5).
    - `FIELD_LENGTH`: Length of the field in meters (default is 500).
    - `FIELD_WIDTH`: Width of the field in meters (default is 250).
    - `WORLD`: Name of the Gazebo world to use (default is `swarmz_world`).

2. Start a game by running the game master launcher in a different terminal:
    ```bash
    cd SWARMZ4/ros2_ws
    source install/setup.bash
    ros2 launch game_master game_master.launch.py
    ```

## Contact
For any questions or feedback, please contact [nicolas.baudesson@alten.com].

"""
Game Master Test Launch File

Description:
    This launch file sets up a complete game testing environment by combining:
    1. Offboard controllers for both teams' drones
    2. Mission control system for waypoint execution
    3. Game master system for game mechanics and scoring

Prerequisites:
    - PX4-Autopilot running (SITL or hardware)
    - ROS 2 environment configured
    - game_master package installed
    - offboard_control_py package installed
    - Proper YAML configurations:
        - spawn_position_test.yaml (drone starting positions)
        - game_master_test.yaml (mission waypoints)
        - game_master configuration (game rules and settings)

Required Packages:
    - offboard_control_py
    - game_master
    - launch
    - launch_ros

Configuration Files:
    - spawn_position_test.yaml: Defines initial drone positions
    - game_master_test.yaml: Defines mission waypoints
    - game_master configs: Define game rules and mechanics

Usage:
    ros2 launch offboard_control_py game_test.launch.py

Expected Behavior:
    1. Spawns NED controllers for all drones in both teams
    2. Initializes mission control system
    3. Starts game master components
    4. Executes defined missions while applying game rules

Notes:
    - Ensure all configuration files are properly set up before launching
    - Monitor console output for initialization status
    - Use game master interfaces for scoring and game control
"""

# Import required ROS 2 launch modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for complete game testing setup.
    
    Returns:
        LaunchDescription: Combined launch description containing:
            - Team 1 offboard controllers
            - Team 2 offboard controllers
            - Mission control node
            - Game master system
    """
    # Get package directories for configuration file access
    pkg_dir = get_package_share_directory('offboard_control_py')
    game_master_pkg_dir = get_package_share_directory('game_master')
    
    # Initialize Team 1's drone controllers
    offboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'offboard_control.launch.py')
        ),
        launch_arguments={
            'team_id': '1',
            'controller_type': 'NED',
            'spawn_file': 'spawn_position_test.yaml'
        }.items()
    )
    
    # Initialize Team 2's drone controllers
    offboard_launch_team2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'offboard_control.launch.py')
        ),
        launch_arguments={
            'team_id': '2',
            'controller_type': 'NED',
            'spawn_file': 'spawn_position_test.yaml'
        }.items()
    )
    
    # Set up mission control for waypoint management
    mission_control = Node(
        package='offboard_control_py',
        executable='offboard_control_client',
        name='mission_control',
        parameters=[{
            'mission_file': 'game_master_test.yaml',
            'log_level': 'info'
        }],
        output='screen'
    )
    
    # Initialize game master system
    game_master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(game_master_pkg_dir, 'launch', 'game_master.launch.py')
        )
    )
    
    # Combine all components in proper initialization order
    return LaunchDescription([
        offboard_launch,      # Must start first to initialize drone controllers
        offboard_launch_team2,
        mission_control,      # Starts after controllers are ready
        game_master_launch    # Game mechanics overlay on top of base system
    ])

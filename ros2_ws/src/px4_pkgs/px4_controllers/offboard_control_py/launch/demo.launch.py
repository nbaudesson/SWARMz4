"""
Full Combat Simulation Demo Launch File

This launch file starts all components of the combat simulation:
1. Game Master - Central controller for the simulation
2. Team 1 Drones (px4_0 through px4_4) - First team's drone controllers
3. Team 2 Drones (px4_5 through px4_9) - Second team's drone controllers

All drones run their individual behavior controllers using the offboard_control system.

Usage:
    ros2 launch offboard_control_py demo.launch.py log_level:=info
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for the full combat simulation."""
    
    # Get package directories
    offboard_pkg_dir = get_package_share_directory('offboard_control_py')
    game_master_pkg_dir = get_package_share_directory('game_master')
    
    # Define launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes: debug, info, warn, error, fatal'
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        log_level_arg,
        
        # Launch Game Master
        LogInfo(msg=["Starting Game Master..."]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(game_master_pkg_dir, 'launch', 'game_master.launch.py')
            ),
            launch_arguments={
                'log_level': LaunchConfiguration('log_level'),
            }.items()
        ),
        
        # Launch Team 1 Drones
        LogInfo(msg=["Starting Team 1 Drones (px4_0 through px4_4)..."]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(offboard_pkg_dir, 'launch', 'offboard_clients.launch.py')
            ),
            launch_arguments={
                'team_id': '1',
                'log_level': LaunchConfiguration('log_level'),
            }.items()
        ),
        
        # Launch Team 2 Drones
        LogInfo(msg=["Starting Team 2 Drones (px4_5 through px4_9)..."]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(offboard_pkg_dir, 'launch', 'offboard_clients.launch.py')
            ),
            launch_arguments={
                'team_id': '2',
                'log_level': LaunchConfiguration('log_level'),
            }.items()
        ),
        
        # Final message
        LogInfo(msg=["All simulation components started. Combat simulation is running."]),
    ])

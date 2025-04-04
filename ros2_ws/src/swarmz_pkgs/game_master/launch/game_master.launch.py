"""
Game Master Launch File
=====================

Components Launched:
-----------------
1. Game Master Node: Central game controller
2. Missile Server: Handles missile firing mechanics
3. Kamikaze Server: Manages self-destruct actions

Configuration:
------------
- Loads parameters from game_master_params.yaml
- Enables simulation time
- Configures debug options for Gazebo tracking
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import signal
import subprocess
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import launch.logging

def shutdown_swarmz():
    """
    Run the shutdown_swarmz.py script to kill all related processes.
    """
    script_path = os.path.join(
        get_package_share_directory('game_master'),
        'utils',
        'shutdown_swarmz.py'
    )
    subprocess.run(['python3', script_path])

def generate_launch_description():
    # Initialize launch logger
    logger = launch.logging.get_logger('launch')
    
    config = os.path.join(
        get_package_share_directory('game_master'),
        'config',
        'game_master_params.yaml'
    )

    # Use proper logging instead of print
    logger.info(f"Loading configuration from: {config}")

    # Add log level configuration
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes: debug, info, warn, error, fatal'
    )
    
    # Replace the original signal handler with a safer shutdown approach
    def safe_shutdown():
        """Safely shutdown the simulation without crashing Gazebo"""
        logger.info("Launch file received shutdown request")
        # Don't kill Gazebo processes directly
        # Only cleanup ROS2 nodes and let them handle their own Gazebo connections
        
    # Register the safer shutdown function
    signal.signal(signal.SIGINT, lambda sig, frame: safe_shutdown())
    
    # Add debug configuration options as launch arguments
    debug_gazebo_arg = DeclareLaunchArgument(
        'debug_gazebo', 
        default_value='false',
        description='Enable Gazebo tracking debug output'
    )

    return LaunchDescription([
        log_level_arg,
        debug_gazebo_arg,
        
        # Add LogInfo actions for clear launch sequence logging
        LogInfo(msg=["Starting Game Master system components..."]),
        
        Node(
            package='game_master',
            executable='game_master_node',
            name='game_master_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[config, {
                'enable_gazebo_debug': LaunchConfiguration('debug_gazebo'),
            }]
        ),
        Node(
            package='game_master',
            executable='missile_server',
            name='missile_server',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[config]
        ),
        Node(
            package='game_master',
            executable='kamikaze_server',
            name='kamikaze_server',
            output='screen', 
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[config]
        ),
        
        # Add completion log message
        LogInfo(msg=["All Game Master components started"]),
    ])

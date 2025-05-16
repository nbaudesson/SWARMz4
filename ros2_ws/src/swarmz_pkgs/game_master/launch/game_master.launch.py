"""
Game Master Launch File
=====================

Components Launched:
-----------------
1. Game Master Node: Central game controller for multi-robot combat simulation

Configuration:
------------
- Loads parameters from game_master_params.yaml
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
    
    # Get the path to the config file
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
    
    # Register shutdown handler
    def safe_shutdown():
        """Safely shutdown the simulation without crashing Gazebo"""
        logger.info("Launch file received shutdown request")
        shutdown_swarmz()
        
    signal.signal(signal.SIGINT, lambda sig, frame: safe_shutdown())

    return LaunchDescription([
        log_level_arg,
        
        # Start the game master node
        LogInfo(msg=["Starting Game Master node..."]),
        
        # Launch the game master node with configuration from YAML only
        Node(
            package='game_master',
            executable='game_master_node',
            name='game_master_node',
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[
                config  # Load all parameters from the YAML file
            ]
        ),
        
        # Completion message
        LogInfo(msg=["Game Master node started"]),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import signal
import subprocess

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
    config = os.path.join(
        get_package_share_directory('game_master'),
        'config',
        'game_master_params.yaml'
    )

    # Debug print
    print("Config file path:", config)

    # Register the shutdown function to be called on termination
    signal.signal(signal.SIGINT, lambda sig, frame: shutdown_swarmz())

    return LaunchDescription([
        Node(
            package='game_master',
            executable='game_master_node',
            name='game_master_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='game_master',
            executable='missile_server',
            name='missile_server',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='game_master',
            executable='kamikaze_server',
            name='kamikaze_server',
            output='screen',
            parameters=[config]
        ),
    ])

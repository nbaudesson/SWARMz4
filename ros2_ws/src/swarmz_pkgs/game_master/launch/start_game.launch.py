from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('game_master'),
        'config',
        'game_master_params.yaml'
    )

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
            executable='game_master_missile_server',
            name='missile_service_server',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='game_master',
            executable='game_master_kamikaze_server',
            name='kamikaze_service_server',
            output='screen',
            parameters=[config]
        ),
    ])

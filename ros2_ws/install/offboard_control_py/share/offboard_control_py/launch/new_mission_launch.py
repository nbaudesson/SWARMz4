import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mission_file = os.path.join(
        get_package_share_directory('offboard_control_py'),
        'missions',
        'new_mission.yaml'
    )

    return LaunchDescription([
        Node(
            package='offboard_control_py',
            executable='mission_control',
            name='mission_control_5',
            namespace='px4_5',
            output='screen',
            parameters=[{'mission': mission_file}]
        ),
        Node(
            package='offboard_control_py',
            executable='mission_control',
            name='mission_control_10',
            namespace='px4_10',
            output='screen',
            parameters=[{'mission': mission_file}]
        )
    ])

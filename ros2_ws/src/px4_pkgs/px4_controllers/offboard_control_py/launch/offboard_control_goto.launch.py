from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control_py',
            executable='offboard_control_goto',
            namespace='px4_1',
            name='offboard_control_goto_px4_1',
            output='screen'
        ),
        Node(
            package='offboard_control_py',
            executable='offboard_control_goto',
            namespace='px4_2',
            name='offboard_control_goto_px4_2',
            output='screen'
        )
    ])

# File: boat_test_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with a default value
    team_id = DeclareLaunchArgument(
        'team_id',
        default_value="1",  # Default value, can be changed by user at launch time
        description='Team ID for the boat_test node'
    )
    tolerance_angle = DeclareLaunchArgument(
        'tolerance_angle',
        default_value="1.0",  # Default value, can be changed by user at launch time
        description='Tolerance for yaw precision'
    )
    tolerance_distance = DeclareLaunchArgument(
        'tolerance_distance',
        default_value="5.0",  # Default value, can be changed by user at launch time
        description='Tolerance for distance precision'
    )
    # Node setup with parameter substitution
    boat_test_node = Node(
        package='boat_driver',
        executable='boat_test',
        name='boat_test',
        output='screen',
        parameters=[{'team_id': LaunchConfiguration('team_id'),
                    'tolerance_angle': LaunchConfiguration('tolerance_angle'),
                    'tolerance_distance': LaunchConfiguration('tolerance_distance')}]
    )

    return LaunchDescription([
        team_id,
        tolerance_angle,
        tolerance_distance,
        boat_test_node
    ])

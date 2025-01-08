from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'share/my_robot_description/sdf/flag_ship.sdf'],
            output='screen'
        ),
    ])

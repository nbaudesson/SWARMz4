from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml_params(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # Declare arguments for number of each robot type
    num_px4_arg = DeclareLaunchArgument(
        'num_px4',
        default_value='2',  # Changed to string
        description='Number of PX4 drones'
    )

    num_flagships_arg = DeclareLaunchArgument(
        'num_flagships',
        default_value='0',  # Changed to string
        description='Number of flag ships'
    )

    # Include game_master launch file
    game_master_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('game_master'),
                'launch',
                'game_master.launch.py'
            )
        )
    )

    # Include game_client launch file with arguments
    game_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('game_master'),
                'launch',
                'game_client.launch.py'
            )
        ),
        launch_arguments={
            'num_px4': LaunchConfiguration('num_px4'),
            'num_flagships': LaunchConfiguration('num_flagships')
        }.items()
    )

    # Return the launch description with all the actions
    return LaunchDescription([
        num_px4_arg,
        num_flagships_arg,
        game_master_launch,
        game_client_launch,
    ])

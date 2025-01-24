from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml

def load_yaml_params(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # Declare arguments for number of each robot type
    num_px4_arg = DeclareLaunchArgument(
        'num_px4',
        default_value='2',  # Corrected to string
        description='Number of PX4 drones'
    )

    num_flagships_arg = DeclareLaunchArgument(
        'num_flagships',
        default_value='0',  # Corrected to string
        description='Number of flag ships'
    )

    # Return the launch description with all the actions
    return LaunchDescription([
        num_px4_arg,
        num_flagships_arg,
        LogInfo(msg="Launching game_client with the following parameters:"),
        LogInfo(msg=["num_px4: ", LaunchConfiguration('num_px4')]),
        LogInfo(msg=["num_flagships: ", LaunchConfiguration('num_flagships')]),
        OpaqueFunction(function=launch_nodes),
        LogInfo(msg="Launch sequence initiated.")
    ])

def launch_nodes(context):
    # Get the number of PX4 drones and flagships from the launch configuration
    num_px4 = int(LaunchConfiguration('num_px4').perform(context))
    num_flagships = int(LaunchConfiguration('num_flagships').perform(context))

    # Create nodes for PX4 drones
    px4_nodes = [
        Node(
            package='game_master',
            executable='game_master_client_node',
            name=f'px4_{i+1}',
            namespace='',
            output='screen',
            parameters=[{'robot_name': f'px4_{i+1}'}],
            prefix=f'gnome-terminal --title="px4_{i+1} weapons controller" --'
        ) for i in range(num_px4)
    ]

    # Create nodes for flagships
    flagship_nodes = [
        Node(
            package='game_master',
            executable='game_master_client_node',
            name=f'flagship_{i+1}',
            namespace='',
            output='screen',
            parameters=[{'robot_name': f'flagship_{i+1}'}],
            prefix=f'gnome-terminal --title="flagship_{i+1} weapons controller" --'
        ) for i in range(num_flagships)
    ]

    # Return the list of nodes to be launched
    return px4_nodes + flagship_nodes

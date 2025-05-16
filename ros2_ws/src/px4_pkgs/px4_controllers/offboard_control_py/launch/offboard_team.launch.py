"""
Multi-Drone Offboard Control Launch File

This launch file enables running multiple PX4 drone controllers with different configurations.
It supports different control frame types (NED, FRD, FLU) and can be configured through command line
arguments or YAML configuration files.

Features:
- Supports multiple drones per team
- Configurable coordinate system (NED/FRD/FLU)
- Spawn position configuration via YAML
- Automatic node namespace management
- Default parameter handling

Usage Examples:
    1. Basic launch with default NED controller and POSITION offboard mode for all drones in team 1:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1

    2. Launch with specific coordinate system and offboard mode for all drones of team 1:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 coordinate_system:=FRD offboard_mode:=velocity

    3. Launch with custom spawn positions and custom configs:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 spawn_file:=path/to/spawn_config.yaml config_file:=path/to/controller_config.yaml

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import os.path
import subprocess


def load_yaml_for_node(config_file_path, node_namespace):
    """
    Load config for each drone from a global config file with all the configs
    """
    with open(config_file_path, 'r') as f:
        all_config = yaml.safe_load(f)
        return all_config[node_namespace] if node_namespace in all_config else ""

def load_yaml(file_path):
    """
    Load and parse a YAML configuration file, ensuring numeric values are float.
    
    Args:
        file_path (str): Path to YAML file
        
    Returns:
        dict: Parsed YAML content with numeric values as float
    """
    def convert_numerics_to_float(data):
        """Recursively convert numeric values to float in nested dictionaries."""
        if isinstance(data, dict):
            return {k: convert_numerics_to_float(v) for k, v in data.items()}
        elif isinstance(data, (list, tuple)):
            return [convert_numerics_to_float(item) for item in data]
        elif isinstance(data, (int, float)):
            return float(data)
        return data

    with open(file_path, 'r') as f:
        yaml_data = yaml.safe_load(f)
        return convert_numerics_to_float(yaml_data)

def resolve_file_path(file_path, package_dir, default_name):
    """
    Resolve file path, checking if absolute or needs to be found in config folder.
    If the file doesn't exist in the default location, check source directory.
    
    Args:
        file_path (str): Path provided by user
        package_dir (str): Path to package root
        default_name (str): Default filename to use in config folder
        
    Returns:
        str: Absolute path to the file
    """
    resolved_path = ""
    
    if not file_path:
        # If no file specified, use default in config folder
        resolved_path = os.path.join(package_dir, 'config', default_name)
    elif os.path.isabs(file_path):
        # If absolute path provided, use it directly
        resolved_path = file_path
    else:
        # If relative path, look in config folder
        resolved_path = os.path.join(package_dir, 'config', file_path)
    
    # Check if file exists at resolved path
    if os.path.exists(resolved_path):
        return resolved_path
    
    # Fallback to source directory path if the file doesn't exist
    # Get SWARMZ4_PATH from environment
    swarmz4_path = os.environ.get('SWARMZ4_PATH')
    if swarmz4_path and default_name == 'spawn_position.yaml':
        source_dir_path = os.path.join(
            swarmz4_path, 
            'ros2_ws/src/px4_pkgs/px4_controllers/offboard_control_py/config', 
            default_name
        )
        if os.path.exists(source_dir_path):
            print(f"Using spawn file from source directory: {source_dir_path}")
            return source_dir_path
    
    # Return the original path even if not found (error will be handled later)
    return resolved_path

def launch_mavros_subprocess(namespace, drone_id, tgt_system):
    # Suppress MAVROS prints by redirecting stdout and stderr to DEVNULL
    mavros_cmd = [
        'ros2', 'launch', 'mavros', 'px4.launch',
        f'fcu_url:=udp://:1454{drone_id}@127.0.0.1:1455{drone_id}',
        f'namespace:={namespace}',
        f'tgt_system:={tgt_system}'
    ]
    subprocess.Popen(mavros_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def generate_drone_nodes(context, *args, **kwargs):
    """Generate ROS2 nodes for each drone based on configuration."""
    # Get launch configurations
    team_id = LaunchConfiguration('team_id').perform(context)
    coordinate_system = LaunchConfiguration('coordinate_system').perform(context)
    offboard_mode = LaunchConfiguration('offboard_mode').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    spawn_file = LaunchConfiguration('spawn_file').perform(context)
    package_dir = get_package_share_directory('offboard_control_py')
    
    # Resolve paths for configuration files
    spawn_file_path = resolve_file_path(spawn_file, package_dir, 'spawn_position.yaml')
    config_file_path = resolve_file_path(config_file, package_dir, 'controller_config.yaml')
    
    # Verify files exist
    if not os.path.exists(spawn_file_path):
        raise FileNotFoundError(f"Spawn file not found: {spawn_file_path}")

    # Load and log spawn positions
    spawn_data = load_yaml(spawn_file_path)
    team_drones = spawn_data.get(team_id, {})
    
    # Process each drone that has spawn data
    nodes = []
    for drone_id, spawn_info in team_drones.items():
        namespace = f'px4_{drone_id}'
        tgt_system = int(drone_id) + 1
        config=None
        # 1. Try launch argument
        if coordinate_system and offboard_mode: 
            config = {'coordinate_sysyem':coordinate_system, 'offboard_mode':offboard_mode}
        elif coordinate_system:
            config = {'coordinate_sysyem':coordinate_system, 'offboard_mode':'position'}
        elif offboard_mode:
            config = {'coordinate_sysyem':'NED', 'offboard_mode':{offboard_mode}}
        # 2. Try controller config file
        if not config and load_yaml_for_node(config_file_path, namespace):
            config = load_yaml_for_node(config_file_path, namespace)
        # 3. Use default
        if not config:
            config = {'coordinate_sysyem':'NED', 'offboard_mode':'position'}

        # Launch MAVROS as subprocess (not as Node)
        launch_mavros_subprocess(namespace, drone_id, tgt_system)

        # Offboard controller node (still as ROS2 node)
        controller_node = Node(
            package='offboard_control_py',
            executable='offboard_control_px4',
            namespace=namespace,
            parameters=[
                config, {
                    'spawn_x': spawn_info.get('x', 0.0),
                    'spawn_y': spawn_info.get('y', 0.0),
                    'spawn_yaw': spawn_info.get('yaw', 0.0)
                }
            ],
            output='screen',
            emulate_tty=True
        )

        # Group: mavros first, then controller after a short delay
        group = GroupAction([
            TimerAction(period=3.0, actions=[controller_node])
        ])
        nodes.append(group)
        
    return nodes

def generate_launch_description():
    """
    Generate launch description with configurable arguments.
    
    Launch Arguments:
    - team_id: Team identifier (1 or 2)
    - config_file: Path to controller configuration YAML
    - spawn_file: Path to spawn positions YAML
    
    Returns:
        LaunchDescription: Configured launch description
    """
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'team_id',
            default_value='1',
            description='Team ID (1 or 2)'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to controller configuration YAML file'
        ),
        DeclareLaunchArgument(
            'spawn_file',
            default_value='',
            description='Path to spawn positions YAML file'
        ),
        DeclareLaunchArgument(
            'coordinate_system',
            default_value='',
            description='Default coordinate system (NED, FRD or FLU) if not specified in config'
        ),
        DeclareLaunchArgument(
            'offboard_mode',
            default_value='',
            description='Default offboard mode if not specified in config'
        ),
        # Generate nodes through OpaqueFunction
        OpaqueFunction(function=generate_drone_nodes)
    ])

# Note:

# OpaqueFunction is a special launch primitive that:
# 1. Delays execution of the wrapped function until launch time
# 2. Provides access to the launch context when executed
# 3. Resolves LaunchConfiguration objects to their actual values
#
# Why "Opaque"?
# - The function's results can't be known until runtime
# - The launch system can't "see through" what the function will do
# - It's a black box from the launch system's perspective
#
# How it works:
# 1. During launch file parsing:
#    - Launch arguments are registered but not resolved
#    - OpaqueFunction is registered but not executed
#
# 2. During launch execution:
#    - Launch arguments are resolved to their final values
#    - OpaqueFunction calls generate_drone_nodes with the launch context
#    - generate_drone_nodes can now access the resolved argument values
#    - The returned nodes are added to the launch description
#
# Benefits:
# - Can make dynamic decisions based on runtime values
# - Can access resolved launch configurations
# - Can generate varying numbers of nodes based on configuration
# - Allows complex node generation logic
#
# Use cases:
# - Dynamic node generation based on configuration files
# - Runtime decision making for launch configuration
# - Complex node setup requiring access to resolved parameters
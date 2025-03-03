"""
Multi-Drone Offboard Control Launch File

This launch file enables running multiple PX4 drone controllers with different configurations.
It supports two control frame types (NED and FRD) and can be configured through command line
arguments or YAML configuration files.

Features:
- Supports multiple drones per team
- Configurable controller types (NED/FRD) # Add new controllers by updating controller_map
- Spawn position configuration via YAML
- Controller type configuration via YAML or launch arguments
- Automatic node namespace management
- Default parameter handling

Usage Examples:
    1. Basic launch with default NED controller for all drones in team 1:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1

    2. Launch with specific controller type for all drones:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 controller_type:=FRD

    3. Launch with custom controller configuration:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 config_file:=path/to/controller_config.yaml

    4. Launch with custom spawn positions:
       ros2 launch offboard_control_py offboard_control.launch.py team_id:=1 spawn_file:=path/to/spawn_config.yaml

Adding a New Controller Type:
1. Create your controller node (e.g., 'offboard_control_custom.py')
2. Add the controller to the controller_map dictionary in generate_drone_nodes():
   controller_map = {
       'NED': 'offboard_control_ned',
       'FRD': 'offboard_control_frd',
       'CUSTOM': 'offboard_control_custom'  # Add your controller here
   }
3. Update the controller configuration YAML to use your new controller type:
   "1": {
       "1": "CUSTOM",  # Use your controller for drone 1
       ...
   }
4. Ensure your controller accepts the same parameters as existing controllers:
   - spawn_x, spawn_y, spawn_yaw
   - takeoff_height, hover_timeout, land_height_threshold
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import os.path

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
    
    Args:
        file_path (str): Path provided by user
        package_dir (str): Path to package root
        default_name (str): Default filename to use in config folder
        
    Returns:
        str: Absolute path to the file
    """
    if not file_path:
        # If no file specified, use default in config folder
        return os.path.join(package_dir, 'config', default_name)
    elif os.path.isabs(file_path):
        # If absolute path provided, use it directly
        return file_path
    else:
        # If relative path, look in config folder
        return os.path.join(package_dir, 'config', file_path)

def generate_drone_nodes(context, *args, **kwargs):
    """Generate ROS2 nodes for each drone based on configuration."""
    # Get launch configurations
    team_id = LaunchConfiguration('team_id').perform(context)
    controller_type = LaunchConfiguration('controller_type').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    spawn_file = LaunchConfiguration('spawn_file').perform(context)
    package_dir = get_package_share_directory('offboard_control_py')
    
    # Resolve paths for configuration files
    spawn_file_path = resolve_file_path(spawn_file, package_dir, 'spawn_position_test.yaml')
    config_file_path = resolve_file_path(config_file, package_dir, 'controller_config.yaml')
    
    # Verify files exist
    if not os.path.exists(spawn_file_path):
        raise FileNotFoundError(f"Spawn file not found: {spawn_file_path}")
    
    # Load spawn positions
    spawn_data = load_yaml(spawn_file_path)
    
    # Load controller configuration if file exists
    controller_config = None
    if config_file and os.path.exists(config_file_path):
        controller_config = load_yaml(config_file_path)
    
    nodes = []
    
    # Get team's drone data
    team_drones = spawn_data.get(team_id, {})
    
    # Map of supported controller types to their executables
    # Add new controllers here when implementing them
    controller_map = {
        'NED': 'offboard_control_ned',  # North-East-Down frame controller
        'FRD': 'offboard_control_frd'   # Forward-Right-Down frame controller
        # 'CUSTOM': 'offboard_control_custom'  # Example of adding a new controller. 
                                               # Don't forget to add it to the setup.py to make an executable.
    }
    
    # Add detailed logging for configuration loading
    print("\nController Configuration Decision Log:")
    print("--------------------------------------")
    
    # Log config file status
    if os.path.exists(config_file_path):
        print(f"Found controller config file: {config_file_path}")
        controller_config = load_yaml(config_file_path)
        configured_drones = sorted(list(controller_config.get(team_id, {}).keys()))
        print(f"Available controller configurations for team {team_id}: {configured_drones}")
    else:
        print(f"No controller config file found at: {config_file_path}")
        print("Will use default or launch argument controller type")
        controller_config = None

    # Log spawn file status
    spawn_drones = sorted(list(team_drones.keys()))
    print(f"\nSpawn positions for team {team_id}: {spawn_drones}")
    
    # Log launch argument controller type
    print(f"Launch argument controller_type: {controller_type or 'Not provided'}")
    
    print(f"\nController Selection for Team {team_id}:")
    print("----------------------------------------")
    
    # Process each drone that has spawn data
    nodes = []
    for drone_id, spawn_info in team_drones.items():
        print(f"\nDrone {drone_id} controller selection:")
        
        # Determine controller type with detailed logging
        drone_controller = None
        
        # 1. Try launch argument
        if controller_type:
            drone_controller = controller_type
            print(f"  → Using controller from launch argument: {controller_type}")
        
        # 2. Try controller config file
        if not drone_controller and controller_config and str(team_id) in controller_config:
            drone_controller = controller_config[str(team_id)].get(str(drone_id))
            if drone_controller:
                print(f"  → Using controller from config file: {drone_controller}")
            else:
                print("  → No controller specified in config file")
        
        # 3. Use default
        if not drone_controller:
            drone_controller = 'NED'
            print("  → Using default controller: NED")
        
        # Map controller type to executable name
        node_executable = controller_map.get(drone_controller.upper())
        if not node_executable:
            print(f"  ⚠ Warning: Unknown controller type '{drone_controller}'")
            continue
            
        # Create node for this drone
        node = Node(
            package='offboard_control_py',
            executable=node_executable,
            namespace=f'px4_{drone_id}',
            parameters=[{
                'spawn_x': spawn_info.get('x', 0.0),
                'spawn_y': spawn_info.get('y', 0.0),
                'spawn_yaw': spawn_info.get('yaw', 0.0),
                'takeoff_height': 1.0,
                'hover_timeout': 10.0,
                'land_height_threshold': 0.4
            }],
            output='screen',
            emulate_tty=True
        )
        nodes.append(node)
    
    print(f"\nCreated {len(nodes)} controller nodes for team {team_id}\n")
    return nodes

def generate_launch_description():
    """
    Generate launch description with configurable arguments.
    
    Launch Arguments:
    - team_id: Team identifier (1 or 2)
    - controller_type: Default controller for all drones if not specified in config
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
            'controller_type',
            default_value='NED',
            description='Default controller type (NED or FRD) if not specified in config'
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
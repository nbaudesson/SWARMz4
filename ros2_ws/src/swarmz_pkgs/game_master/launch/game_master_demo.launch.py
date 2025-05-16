"""
This launch file runs the Game Master system demonstration, including:
- The game_master_node to manage the simulation combat.
- Offboard control nodes for each drone, configured via YAML files.
- A demo client to orchestrate a choreographed demonstration.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(file_path):
    """Load YAML file and return its contents."""
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return {}

def resolve_file_path(file_path, package_dir, default_name):
    """Resolve file path, checking if absolute or needs to be found in config folder."""
    if not file_path:
        return os.path.join(package_dir, 'config', default_name)
    elif os.path.isabs(file_path):
        return file_path
    else:
        return os.path.join(package_dir, 'config', file_path)

def load_yaml_for_node(config_file_path, node_namespace):
    """Load specific node configuration from a YAML file."""
    try:
        with open(config_file_path, 'r') as f:
            all_config = yaml.safe_load(f)
            if node_namespace in all_config:
                print(f"  → Using controller from config file for {node_namespace}: {all_config[node_namespace]}")
                return all_config[node_namespace]
            else:
                print(f"  → No controller specified in config file for {node_namespace}")
                return {}
    except Exception as e:
        print(f"Error loading controller config: {e}")
        return {}

def generate_launch_description():
    """
    Launch file to demonstrate the Game Master system capabilities.
    Parameters for nodes are loaded from YAML files.
    """
    # Get package directory
    offboard_pkg_dir = get_package_share_directory('offboard_control_py')
    game_master_pkg_dir = get_package_share_directory('game_master')

    # Resolve paths for configuration files
    spawn_file_path = resolve_file_path('', offboard_pkg_dir, 'spawn_position.yaml')
    controller_config_path = resolve_file_path('', offboard_pkg_dir, 'controller_config.yaml')
    game_master_params_path = resolve_file_path('', game_master_pkg_dir, 'game_master_params.yaml')

    # Load spawn positions
    spawn_data = load_yaml(spawn_file_path)

    # Launch the game master node with parameters from a params file
    game_master_node = Node(
        package='game_master',
        executable='game_master_node',
        name='game_master_node',
        output='screen',
        emulate_tty=True,
        parameters=[game_master_params_path]
    )

    # Launch offboard control nodes for each drone
    offboard_control_nodes = []
    for team_id, drones in spawn_data.items():
        for drone_id, spawn_info in drones.items():
            namespace = f'px4_{drone_id}'
            print(f"{namespace} spawn position: {spawn_info}")
            
            # Get controller configuration from config file
            controller_config = load_yaml_for_node(controller_config_path, namespace)
            
            # Use defaults if no config found
            if not controller_config:
                controller_config = {
                    'coordinate_system': 'NED',
                    'offboard_mode': 'position'
                }
                print(f"  → Using default config for {namespace}: coordinate_system: NED, offboard_mode: position")
            
            # Create spawn position parameters
            spawn_params = {
                'spawn_x': float(spawn_info.get('x', 0.0)),
                'spawn_y': float(spawn_info.get('y', 0.0)),
                'spawn_z': 0.0,  # Drones spawn at ground level
                'spawn_yaw': float(spawn_info.get('yaw', 0.0))
            }
            
            # Create offboard control node
            offboard_control_nodes.append(
                Node(
                    package='offboard_control_py',
                    executable='offboard_control_px4',
                    name=f'offboard_control_{drone_id}',
                    namespace=f'/px4_{drone_id}',
                    output='screen',
                    emulate_tty=True,
                    parameters=[controller_config, spawn_params]
                )
            )

    # Launch the demo client with a delay to ensure services are ready
    demo_client = TimerAction(
        period=10.0,  # 10-second delay to ensure all nodes are ready
        actions=[
            Node(
                package='game_master',
                executable='game_master_client_dynamic_test',
                name='game_master_demo',
                output='screen',
                emulate_tty=True
            )
        ]
    )

    # Create the complete launch description
    launch_description = [
        game_master_node,
    ]

    # Add all offboard control nodes
    launch_description.extend(offboard_control_nodes)

    # Add demo client (with delay)
    launch_description.append(demo_client)

    return LaunchDescription(launch_description)

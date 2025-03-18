from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(file_path):
    """Load YAML file and return its contents"""
    # Print the full path for debugging
    print(f"Loading spawn positions from: {file_path}")
    
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
            # Print the loaded data for verification
            print(f"Loaded data: {data}")
            return data
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return {}

def generate_launch_description():
    """
    Launch file to demonstrate the Game Master system capabilities.
    
    This launches:
    1. The game_master_node (manages the simulation combat)
    2. The missile_server (provides missile firing capabilities)
    3. The kamikaze_server (provides kamikaze attack capabilities)
    4. Offboard control nodes for each drone (provides position control)
    5. The demo client (after a delay to ensure services are ready)
    
    The demo client will orchestrate a choreographed demonstration of multiple
    PX4 drones showing formation flying, communications, detections, and
    weapons system capabilities.
    """
    # Get paths
    offboard_pkg_dir = get_package_share_directory('offboard_control_py')
    spawn_file_path = os.path.join(offboard_pkg_dir, 'config', 'spawn_position.yaml')
    
    # Load spawn positions from YAML file
    spawn_data = load_yaml(spawn_file_path)
    
    # Create a unified dictionary of all drones' spawn positions
    drone_spawn_positions = {}
    for team_id in ['1', '2']:
        if team_id in spawn_data:
            for drone_id, position in spawn_data[team_id].items():
                drone_spawn_positions[int(drone_id)] = {
                    'x': position.get('x', 0.0),
                    'y': position.get('y', 0.0),
                    'yaw': position.get('yaw', 0.0)
                }
                # Debug print for each drone's position
                print(f"Configured position for drone {drone_id}: x={position.get('x', 0.0)}, y={position.get('y', 0.0)}, yaw={position.get('yaw', 0.0)}")
    
    # Launch the game master node
    game_master_node = Node(
        package='game_master',
        executable='game_master_node',
        name='game_master_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'drone_detection_range': 20.0},
            {'drone_communication_range': 25.0},
            {'game_duration': 180},  # 5 minutes game duration
        ]
    )
    
    # Launch the missile server
    missile_server = Node(
        package='game_master',
        executable='missile_server',
        name='missile_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'drone_missile_range': 100.0, # Range in meters for drone missile
             'laser_width' : 2.0},  
        ]
    )
    
    # Launch the kamikaze server
    kamikaze_server = Node(
        package='game_master',
        executable='kamikaze_server',
        name='kamikaze_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'explosion_range': 17.0},  # Explosion radius in meters
        ]
    )
    
    # Launch offboard control nodes for each drone
    offboard_control_nodes = []
    for i in range(1, 11):  # For drones px4_1 through px4_10
        # Get spawn position from loaded configuration or use defaults
        spawn_config = drone_spawn_positions.get(i, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        
        # Print the actual parameters being passed
        print(f"Setting spawn for px4_{i}: x={spawn_config['x']}, y={spawn_config['y']}, yaw={spawn_config['yaw']}")
        
        offboard_control_nodes.append(
            Node(
                package='offboard_control_py',
                executable='offboard_control_ned',
                name=f'offboard_control_{i}',
                namespace=f'/px4_{i}',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {
                        'spawn_x': float(spawn_config['x']),
                        'spawn_y': float(spawn_config['y']),
                        'spawn_z': 0.0,  # Drones spawn at ground level
                        'spawn_yaw': float(spawn_config['yaw']),
                        'takeoff_height': 5.0,
                        'hover_timeout': 30.0,  # Longer hover timeout for demo
                        'land_height_threshold': 0.4
                    }
                ]
            )
        )
    
    # Launch the demo client with a delay to ensure services are ready
    demo_client = TimerAction(
        period=10.0,  # 10 second delay to ensure all nodes are ready
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
        missile_server,
        kamikaze_server,
    ]
    
    # Add all offboard control nodes
    launch_description.extend(offboard_control_nodes)
    
    # Add demo client (with delay)
    launch_description.append(demo_client)
    
    return LaunchDescription(launch_description)

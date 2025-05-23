"""
Team Client Launch File
=====================
 
This file launches all control nodes for a team's vehicles:
- 5 Drone controller nodes with different roles
- 1 Boat controller node

Each node represents a place to implement custom behavior for the respective vehicle.
"""
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def create_team_nodes(context, *args, **kwargs):
    """Create team's drone and boat nodes based on team ID."""
    team_id = LaunchConfiguration('team_id').perform(context)
    
    # Calculate namespace base index based on team_id
    base_idx = 0 if team_id == '1' else 5
    
    # Define namespace for each drone position
    namespaces = [f'px4_{base_idx+i}' for i in range(5)]
    
    # Create nodes for this team
    return [
        # === DRONE NODES ===
        # Drone 0: Reconnaissance & Attack
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{base_idx}',
            name=f'drone_{base_idx}_controller',
            namespace=namespaces[0],
            output='screen',
            emulate_tty=True
        ),
        
        # Drone 1: Communications Support
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{base_idx+1}',
            name=f'drone_{base_idx+1}_controller',
            namespace=namespaces[1],
            output='screen',
            emulate_tty=True
        ),
        
        # Drone 2: Kamikaze Drone
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{base_idx+2}',
            name=f'drone_{base_idx+2}_controller',
            namespace=namespaces[2],
            output='screen',
            emulate_tty=True
        ),
        
        # Drone 3: Ship Escort (Front)
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{base_idx+3}',
            name=f'drone_{base_idx+3}_controller',
            namespace=namespaces[3],
            output='screen',
            emulate_tty=True
        ),
        
        # Drone 4: Formation Support
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{base_idx+4}',
            name=f'drone_{base_idx+4}_controller',
            namespace=namespaces[4],
            output='screen',
            emulate_tty=True
        ),
        
        # === BOAT NODE ===
        # Team's flagship/ship controller
        Node(
            package='boat_driver',
            executable=f'boat_client_demo_{team_id}',
            name=f'ship_demo_{team_id}',
            namespace=f'flag_ship_{team_id}',
            output='screen'
        ),
    ]

def generate_launch_description():
    """Generate launch description for team's vehicles."""

    # Declare argument
    team_id_arg = DeclareLaunchArgument(
        'team_id',
        default_value='1',
        description='Team ID (1 or 2)'
    )

    return LaunchDescription([
        team_id_arg,
        LogInfo(msg=["Starting Team ", LaunchConfiguration('team_id'), " vehicles..."]),
        
        # Launch all team nodes with proper namespaces based on team ID
        OpaqueFunction(function=create_team_nodes),
        
        LogInfo(msg=["All team vehicles started and running."]),
    ])
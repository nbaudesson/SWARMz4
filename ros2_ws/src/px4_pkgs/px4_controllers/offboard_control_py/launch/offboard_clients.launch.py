"""
Drone Team Behavior Launch File

This launch file starts all the individual drone behavior controllers for the demo:
- Drone 0: Reconnaissance & Attack
- Drone 1: Communications Support
- Drone 2: Kamikaze Drone
- Drone 3: Ship Escort (Front)
- Drone 4: Formation Support

Each drone runs its own specialized behavior node that implements its specific role.

Usage:
    ros2 launch offboard_control_py offboard_clients.launch.py team_id:=1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def create_client_nodes(context, *args, **kwargs):
    """Create individual client nodes with namespaces based on team ID."""
    team_id = LaunchConfiguration('team_id').perform(context)
    
    # Calculate namespace base index based on team_id
    base_idx = 0 if team_id == '1' else 5
    
    # Define namespace for each drone position
    namespaces = [f'px4_{base_idx+i}' for i in range(5)]
    
    # Create each node individually, with the specific behavior implementation
    return [
        # Drone 0: Reconnaissance & Attack
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{0}',  # Use client 0 from team_1 directory
            name=f'drone_{base_idx}_controller',
            namespace=namespaces[0],
            output='screen',
            emulate_tty=True
        ),
        # Drone 1: Communications Support
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{1}',  # Use client 1 from team_1 directory
            name=f'drone_{base_idx+1}_controller', 
            namespace=namespaces[1],
            output='screen',
            emulate_tty=True
        ),
        # Drone 2: Kamikaze Drone
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{2}',  # Use client 2 from team_1 directory
            name=f'drone_{base_idx+2}_controller',
            namespace=namespaces[2],
            output='screen',
            emulate_tty=True
        ),
        # Drone 3: Ship Escort (Front)
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{3}',  # Use client 3 from team_1 directory
            name=f'drone_{base_idx+3}_controller',
            namespace=namespaces[3],
            output='screen',
            emulate_tty=True
        ),
        # Drone 4: Formation Support
        Node(
            package='offboard_control_py',
            executable=f'offboard_control_client_{4}',  # Use client 4 from team_1 directory
            name=f'drone_{base_idx+4}_controller',
            namespace=namespaces[4],
            output='screen',
            emulate_tty=True
        )
    ]

def generate_launch_description():
    """Generate the launch description for all drone controllers."""
    pkg_dir = get_package_share_directory('offboard_control_py')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'team_id',
            default_value='1',
            description='Team ID (1 or 2)'
        ),
        
        # Include the offboard_team.launch.py (controllers and MAVROS nodes)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'offboard_team.launch.py')
            ),
            launch_arguments={'team_id': LaunchConfiguration('team_id')}.items()
        ),
        
        # Launch individual client nodes based on team ID
        OpaqueFunction(function=create_client_nodes)
    ])

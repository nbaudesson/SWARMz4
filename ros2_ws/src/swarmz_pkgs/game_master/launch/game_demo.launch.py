"""
This launch file runs the Game Master system demonstration, including:
- The game_master_node to manage the simulation combat.
- The client programs launcher, one for each team, each containing 5 drones and a ship.
"""
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():
    """Generate launch description for the full combat simulation."""
    
    # Get package directories
    game_master_pkg_dir = get_package_share_directory('game_master')
    
    # Define launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes: debug, info, warn, error, fatal'
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        log_level_arg,
        
        
        # Launch Team 1 robots
        LogInfo(msg=["Launching team 1 clients"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(game_master_pkg_dir, 'launch', 'game_demo_client_1.launch.py')
            ),
            launch_arguments={'team_id': '1'}.items()
        ),
        
        # Launch Team 2 robots
        LogInfo(msg=["Launching team 2 clients"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(game_master_pkg_dir, 'launch', 'game_demo_client_2.launch.py')
            ),
            launch_arguments={'team_id': '2'}.items()
        ),
        
        # Add a 5-second delay
        TimerAction(
            period=5.0,
            actions=[
                # Launch Game Master node after delay
                
                LogInfo(msg=["Starting game master"]),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(game_master_pkg_dir, 'launch', 'game_master.launch.py')
                    )
                ),
            ]
        ),

        # Final message
        LogInfo(msg=["All simulation components started. Combat simulation is running."]),
    ])
"""
Game Server Launch File
=====================
 
Components Launched:
-----------------

1. Offboard servers for the 5 drones of each teams: Allow to control the drones
2. Cannon action server for both ships: Allow to control the cannon of the ships

"""
 
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import LogInfo,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():

    offboard_pkg_dir = get_package_share_directory('offboard_control_py')
    
    return LaunchDescription([
        
        # Launch Team 1 drone servers
        LogInfo(msg=["Launching Team 1 Drones (px4_0 through px4_4)..."]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(offboard_pkg_dir, 'launch', 'offboard_team.launch.py')
            ),
            launch_arguments={
                'team_id': '1'
            }.items()
        ),
        # Launch Team 2 drone servers
        LogInfo(msg=["Launching Team 2 Drones (px4_5 through px4_9)..."]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(offboard_pkg_dir, 'launch', 'offboard_team.launch.py')
            ),
            launch_arguments={
                'team_id': '2'
            }.items()
        ),
        
        # Launch Team 1 ship cannon server
        Node(
            package='boat_driver',
            executable='cannon_server',
            name='cannon_server_1',
            namespace='flag_ship_1',
            output='screen',
        ),
 
        # Launch Team 2 ship cannon server
        Node(
            package='boat_driver',
            executable='cannon_server',
            name='cannon_server_2',
            namespace='flag_ship_2',
            output='screen',
        ),
        
        # Add completion log message
        LogInfo(msg=["All servers started"]),
    ])
 
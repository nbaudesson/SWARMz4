import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the share directory of the package
    pkg_share = FindPackageShare('flag_ship').find('flag_ship')
    
    # Construct the URDF file path
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'flag_ship.urdf')

    # Read the URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Define parameters
    params = {'robot_description': robot_desc}

    # Node for robot_state_publisher
    rsp = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params]
    )

    # Node for RViz
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return launch.LaunchDescription([rsp, rviz])

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, FindExecutable


def generate_processes(context, *args, **kwargs):
    nb_of_drones = int(LaunchConfiguration('nb_of_drones').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)

    processes = []
    for i in range(1, nb_of_drones + 1):
        cmd = [
            FindExecutable(name='ros2'),
            'run',
            'ros_gz_bridge',
            'parameter_bridge',
            f'/world/default/model/x500_lidar_front_{i}/link/lidar_sensor_link/sensor/lidar/scan'
            f'@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '--ros-args',
            '--remap',
            f'/world/default/model/x500_lidar_front_{i}/link/lidar_sensor_link/sensor/lidar/scan:=/px4_{i}/laser_beam',
            '--log-level',
            log_level,
        ]

        processes.append(
            ExecuteProcess(
                cmd=cmd,
                output='screen',
                shell=True,
            )
        )

    return processes


def generate_launch_description():
    # Declare launch arguments
    declare_nb_of_drones_cmd = DeclareLaunchArgument(
        'nb_of_drones',
        default_value='10',
        description='Number of drones to simulate',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the nodes',
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the declared arguments
    ld.add_action(declare_nb_of_drones_cmd)
    ld.add_action(declare_log_level_cmd)

    # Dynamically generate and add processes
    ld.add_action(OpaqueFunction(function=generate_processes))

    return ld

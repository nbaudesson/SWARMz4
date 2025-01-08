#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction

def generate_launch_description():
    #
    # ARGS
    #

    # number_of_drones = LaunchConfiguration("d")
    # number_of_drones_cmd = DeclareLaunchArgument(
    #     "d",
    #     default_value="1",
    #     description="Number of drones you want to spawn")
    
    headless = LaunchConfiguration("headless")
    headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="0",
        description="Run Gazebo headless")

    px4_path = LaunchConfiguration("px4")
    px4_path_cmd = DeclareLaunchArgument(
        "px4",
        default_value="~/swarmz4/PX4-Autopilot",
        description="PX4-Autopilot directory path")

    latitude = LaunchConfiguration("lat")
    latitude_cmd = DeclareLaunchArgument(
        "lat",
        default_value="43.13471",
        description="Latitude value for PX4 Home")
    
    longitude = LaunchConfiguration("lon")
    longitude_cmd = DeclareLaunchArgument(
        "lon",
        default_value="6.01507",
        description="Longitude value for PX4 Home")
    
    altitude = LaunchConfiguration("alt")
    altitude_cmd = DeclareLaunchArgument(
        "alt",
        default_value="6",
        description="Altitude value for PX4 Home")

    #
    # NODES
    #
    
    # simu_node=Node(
    #         package='swarmz_control',
    #         namespace='px4_offboard',
    #         executable='simu_processes',
    #         name='simu_processes',
    #         prefix='gnome-terminal --',
    #         parameters=[{"d": number_of_drones,
    #                      "px4": px4_path,
    #                      "lat": latitude,
    #                      "lon": longitude,
    #                      "alt": altitude}]
    #     )

    simu_node=Node(
        package='offboard_control_py',
        namespace='px4_offboard',
        executable='simu_circle',
        name='simu_circle',
        # prefix='gnome-terminal --',
        parameters=[{"headless": headless,
                        "px4": px4_path,
                        "lat": latitude,
                        "lon": longitude,
                        "alt": altitude}]
    )
    
    offboard_control_1_node=Node(
            package='offboard_control_cpp',
            namespace='px4_1',
            executable='offboard_control',
            name='offboard_control_1',
        )
    
    offboard_control_2_node=Node(
            package='offboard_control_cpp',
            namespace='px4_2',
            executable='offboard_control',
            name='offboard_control_2',
        )

    offboard_control_3_node=Node(
            package='offboard_control_cpp',
            namespace='px4_3',
            executable='offboard_control',
            name='offboard_control_3',
        )
    
    offboard_control_4_node=Node(
            package='offboard_control_cpp',
            namespace='px4_4',
            executable='offboard_control',
            name='offboard_control_4',
        )
    
    offboard_control_5_node=Node(
            package='offboard_control_cpp',
            namespace='px4_5',
            executable='offboard_control',
            name='offboard_control_5',
        )

    # for i in NB_OF_DRONES:
    #     offboard_control=Node(
    #             package='px4_ros_com',
    #             namespace='px4_'+str(i),
    #             executable='offboard_control',
    #             name='offboard_control_'+str(i),
    #     )


    ld = LaunchDescription()

    ld.add_action(headless_cmd)
    ld.add_action(px4_path_cmd)
    ld.add_action(latitude_cmd)
    ld.add_action(longitude_cmd)
    ld.add_action(altitude_cmd)

    ld.add_action(simu_node)
    ld.add_action(TimerAction(period=35.0,actions=[offboard_control_1_node, offboard_control_2_node, offboard_control_3_node, offboard_control_4_node, offboard_control_5_node]))
    # ld.add_action(TimerAction(period=35.0,actions=[offboard_control_1_node]))

    # for i in NB_OF_DRONES:
    #     ld.add_action(offboard_control)

    return ld

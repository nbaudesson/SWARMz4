# PX4 Gazebo Bridge

This package provides ROS2 launch files and configurations to bridge PX4 simulation data between ROS2 and Gazebo.

### This README provides:

Brief overview of the package
Instructions for using the existing laser scanner bridge
Clear steps for adding new bridges with examples
Reference to the supported message types
The main configuration components are already in the files shown earlier:

The launch file (px4_laser_gz_bridge.launch.py) handles the bridge setup
The YAML config (px4_gz_bridge_params.yaml) defines the bridge parameters
To add more bridges, you would follow the same pattern but with different topics and message types as needed.

## Current Bridges

### Laser Scanner Bridge
Bridges laser scanner data from Gazebo to ROS2 for multiple drones. The bridge maps Gazebo's laser scan messages to ROS2's `sensor_msgs/msg/LaserScan` type.

To launch the laser scanner bridge:
```bash
ros2 launch px4_gz_bridge px4_laser_gz_bridge.launch.py nb_of_drones:=<number_of_drones>
```

Parameters:
- `nb_of_drones` (default: 10): Number of drones to bridge
- `log_level` (default: info): Logging level for the nodes

## Adding New Bridges

To add new bridges:

1. Create a new YAML configuration in `config/` directory following this template:
```yaml
- ros_topic_name: "<ros_topic_name>"
  gz_topic_name: "<gz_topic_name>"
  ros_type_name: "<ros_message_type>"
  gz_type_name: "<gz_message_type>"
  direction: GZ_TO_ROS  # or ROS_TO_GZ or BIDIRECTIONAL
```

2. Create a new launch file in `launch/` directory:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['<topic>@<ros_msg_type>@<gz_msg_type>'],
            remappings=[('<original_topic>', '<remapped_topic>')],
        )
    ])
```

## Supported Message Types
For a complete list of supported message type mappings between ROS2 and Gazebo, refer to the [ros_gz_bridge documentation](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments for number of each robot type
    num_px4_arg = DeclareLaunchArgument(
        'num_px4',
        default_value='2',
        description='Number of PX4 drones'
    )

    num_flagships_arg = DeclareLaunchArgument(
        'num_flagships',
        default_value='0',
        description='Number of flag ships'
    )

    # Create lists to store nodes
    px4_commands = []
    flagship_commands = []

    # Create PX4 nodes
    num_px4 = int(LaunchConfiguration('num_px4').perform())
    for i in range(num_px4):
        px4_commands.append(f'--tab --title="px4_{i+1}" -e "bash -c \'ros2 run game_master game_master_client.py --ros-args -p robot_name:=px4_{i+1}; exec bash\'"')

    # Create flagship nodes
    num_flagships = int(LaunchConfiguration('num_flagships').perform())
    for i in range(num_flagships):
        flagship_commands.append(f'--tab --title="flagship_{i+1}" -e "bash -c \'ros2 run game_master game_master_client.py --ros-args -p robot_name:=flagship_{i+1}; exec bash\'"')

    processes = [
        ExecuteProcess(
            cmd=['gnome-terminal', '--'] + px4_commands,
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal', '--'] + flagship_commands,
            output='screen'
        )
    ]

    return LaunchDescription([
        num_px4_arg,
        num_flagships_arg
    ] + processes)

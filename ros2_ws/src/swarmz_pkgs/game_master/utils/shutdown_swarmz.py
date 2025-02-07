import subprocess

def shutdown_ros2_and_gazebo():
    """
    Shutdown all ROS 2 and Gazebo related processes.
    """
    processes_to_kill = [
        "gz sim",
        "ros_gz_bridge"
    ]

    for process in processes_to_kill:
        try:
            subprocess.run(f'pkill -f "{process}"', shell=True, check=True)
            print(f'Successfully killed {process} processes.')
        except subprocess.CalledProcessError as e:
            print(f'Failed to kill {process} processes: {e}')

    ros2_nodes_to_kill = [
        "game_master_node",
        "missile_server",
        "kamikaze_server"
    ]

    for node in ros2_nodes_to_kill:
        try:
            subprocess.run(f'pgrep -f "\-r __node:={node}" | xargs kill -9', shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f'Failed to kill ros2 node referenced to {node} : {e}')


if __name__ == "__main__":
    shutdown_ros2_and_gazebo()

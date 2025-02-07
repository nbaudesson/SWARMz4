import subprocess
import sys
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node

def get_model_id(model_name):
    """
    Find the ID of a model by its name using Gazebo Transport API.
    
    :param model_name: The name of the model.
    :return: The ID of the model if found, otherwise None.
    """
    node = Node()
    scene_info = Scene()

    # Request scene info from Gazebo
    result, response = node.request("/world/default/scene/info", Empty(), Empty, Scene, 1000)
    if not result:
        print("Failed to send scene info request.")
        return None

    scene_info = response

    # Search for the model in the received scene info
    for model in scene_info.model:
        if model.name == model_name:
            print(f"Found model '{model_name}' with ID: {model.id}")
            return model.id

    print(f"Model '{model_name}' not found.")
    return None

def remove_model(model_id):
    """
    Remove the model using its ID via Gazebo Transport API.
    
    :param model_id: The ID of the model to be removed.
    """
    node = Node()
    entity_msg = Entity()
    entity_msg.id = model_id
    entity_msg.type = Entity.MODEL  # Type 1 for models

    # Request to remove the model from Gazebo
    result, response = node.request("/world/default/remove", entity_msg, Entity, Boolean, 1000)
    if result and response.data:
        print(f"Successfully requested removal of model ID {model_id}")
    else:
        print(f"Failed to request removal of model ID {model_id}")

def kill_drone_processes(instance_number):
    """
    Kill the processes of all ROS 2 nodes running on the concerned px4_ instance.
    
    :param instance_number: The instance number of the drone.
    """
    namespace = f'/px4_{instance_number}'
    print(f'Killing processes of all ROS 2 nodes in namespace {namespace}')
    try:
        # Kill ros2 processes in the namespace
        try:
            subprocess.run(f'pgrep -f "\-r __ns:={namespace}" | xargs kill -9', shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f'Failed to kill ros2 processes referenced to {namespace} : {e}')
        
        # Kill the px4 processes
        try:
            subprocess.run(f'pkill -f "px4 -i {instance_number}"', shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f'Failed to kill px4 processes referenced to {instance_number} : {e}')
        
        # Kill any other px4 related processes
        try:
            subprocess.run(f'pkill -f "px4_{instance_number}"', shell=True, check=True)
        except subprocess.CalledProcessError as e:
            print(f'Failed to kill any px4_{instance_number} related processes : {e}')
    except subprocess.CalledProcessError as e:
        print(f'Failed to kill processes in namespace {namespace}: {e}')

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python kill_drone.py <instance_number>")
        sys.exit(1)
    
    instance_number = int(sys.argv[1])
    model_name = f"x500_lidar_front_{instance_number}"

    # Kill the drone processes
    kill_drone_processes(instance_number)

    # Remove the drone model from Gazebo
    model_id = get_model_id(model_name)
    if model_id:
        remove_model(model_id)

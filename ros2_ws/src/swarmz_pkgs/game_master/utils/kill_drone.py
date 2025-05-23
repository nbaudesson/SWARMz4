import subprocess
import sys
import threading
import time
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node
import os
from ament_index_python.packages import get_packages_with_prefixes, get_package_share_directory
import logging 

def get_model_id(model_name, logger=None, world_name="game_world", gz_node=None):
    """
    Find the ID of a model by its name using Gazebo Transport API.
    Falls back to command line if the API request fails.
    
    :param model_name: The name of the model.
    :param world_name: The name of the Gazebo world.
    :param gz_node: Optional existing Gazebo transport node to use
    :return: The ID of the model if found, otherwise None.
    """
    logger = logger or logging.getLogger(__name__)

    # Try using the Gazebo Transport API first
    node = gz_node if gz_node is not None else Node()
    scene_info = Scene()

    # Request scene info from Gazebo using the provided world name
    try:
        result, response = node.request(f"/world/{world_name}/scene/info", Empty(), Empty, Scene, 1000)
        if result:
            scene_info = response
            # Search for the model in the received scene info
            for model in scene_info.model:
                if model.name == model_name:
                    return model.id
        else:
            logger.warning(f"Gazebo Transport API request failed: No response received.")
    except Exception as e:
        logger.error(f"Gazebo Transport API request failed with exception: {e}")
    
    # # If we get here, the API request failed or the model wasn't found
    # # Try using the working gz model command as fallback
    # try:
    #     # Construct the command
    #     cmd = ["gz", "model", "-m", model_name]
    #     logger.info(f"Executing command: {' '.join(cmd)}")

    #     # Execute the command
    #     result = subprocess.run(cmd, capture_output=True, text=True)
    #     # logger.info(f"Command output: {result.stdout}")
    #     # logger.info(f"Command error (if any): {result.stderr}")

    #     # Process the output
    #     model_id = extract_model_id(result.stdout, model_name, logger)
    #     if model_id is not None:
    #         logger.info(f"Found model '{model_name}' with ID: {model_id} (via gz model -m)")
    #         return model_id
    #     else:
    #         logger.warning(f"Model ID for '{model_name}' not found in command output.")
    #         return None

    # except Exception as e:
    #     logger.error(f"Command line fallback failed with exception: {e}")
        return None

def extract_model_id(output, model_name, logger=None):
    """
    Extract the model ID from the command output of `gz model -m`.

    Args:
        output (str): The command output.
        model_name (str): The name of the model.

    Returns:
        int: The model ID if found, otherwise None.
    """
    logger = logger or logging.getLogger(__name__)
    
    try:
        # Look for the line containing the model name and ID
        lines = output.splitlines()
        for i, line in enumerate(lines):
            if f"Name: {model_name}" in line:
                # The model ID is typically on the line above "Name: <model_name>"
                if i > 0 and "Model: [" in lines[i - 1]:
                    model_id_str = lines[i - 1].split("[")[1].split("]")[0]
                    return int(model_id_str)
    except Exception as e:
        logger.error(f"Error extracting model ID: {e}")
    return None

def remove_model(model_id, fail_event, logger=None, world_name="game_world", gz_node=None):
    """
    Remove the model using its ID via Gazebo Transport API.
    Falls back to command line if the API request fails.
    
    :param model_id: The ID of the model to be removed.
    :param world_name: The name of the Gazebo world.
    :param gz_node: Optional existing Gazebo transport node to use
    """
    logger = logger or logging.getLogger(__name__)

    # Try using the Gazebo Transport API first
    node = gz_node if gz_node is not None else Node()
    entity_msg = Entity()
    entity_msg.id = model_id
    entity_msg.type = Entity.MODEL  # Type 1 for models

    # Track if we've successfully removed the model
    success = False

    # Request to remove the model from Gazebo using the provided world name
    try:
        result, response = node.request(f"/world/{world_name}/remove", entity_msg, Entity, Boolean, 100)
        if result and response.data:
            success = True
    except Exception as e:
        logger.error(f"Gazebo Transport API removal failed: {e}")

       # If the first method fails, attempt to remove the model using raw request
        entity_byte = entity_msg.SerializeToString()
        try:
            time.sleep(0.5)
            result, response = node.request_raw(
                f"/world/{world_name}/remove",
                entity_byte,
                "gz.msgs.Entity",
                "gz.msgs.Boolean",  # Correct "Boolan" to "Boolean" if needed
                100
            )

            if result and response:
                success = True
        except Exception as e:
            logger.error(f"Gazebo Transport API removal (raw) failed: {e}")
    
    # If no method succeeded, indicate failure
    if not success:
        fail_event.set()  # Signal that the thread to remove the model as failed 

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
        if instance_number != 0:
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

def kill_processes_thread(instance_number):
    """
    Kill drone processes in a separate thread
    
    :param instance_number: The instance number of the drone
    """
    try:
        kill_drone_processes(instance_number)
    except Exception as e:
        print(f"Error killing processes for drone {instance_number}: {e}")

def remove_model_thread(model_id, fail_event, world_name="game_world", gz_node=None):
    """
    Remove model in a separate thread
    
    :param model_id: The ID of the model to be removed
    :param world_name: The name of the Gazebo world
    :param gz_node: Optional existing Gazebo transport node to use
    """
    try:
        remove_model(model_id, fail_event, world_name, gz_node)
    except Exception as e:
        print(f"Error removing model {model_id}: {e}")


def get_own_package_name():
    current_path = os.path.abspath(__file__)
    packages = get_packages_with_prefixes()

    for pkg_name, install_prefix in packages.items():
        # Vérifie si le fichier courant est dans le répertoire du package
        if current_path.startswith(os.path.abspath(install_prefix)):
            return pkg_name

    raise RuntimeError("Impossible de détecter automatiquement le nom du package.")
                       
def shutdown_drone(instance_number, logger=None):
    """
    Shutdown the drown in QGC and stop all the mavlink interfaces

    :param instance_number : The ID of the drone to be removed
    """
    package_name = get_own_package_name()
    script_path = os.path.join(get_package_share_directory(package_name), "utils", "shutdown.sh")


    try:
        result = subprocess.run(
            ["expect", script_path, f"px4_{instance_number}"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
    except subprocess.CalledProcessError as e:
        logger.error(f"Error shutting down the drone '{instance_number}':\n{e.stderr}", file=sys.stderr)
        sys.exit(e.returncode)

def kill_drone_from_game_master_thread(namespace, drone_model_base_name, drone_models={}, logger=None, world_name="game_world", gz_node=None):
    """
    Thread target function that performs the actual drone kill procedure.
    
    Args:
        Same as kill_drone_from_game_master
    """
    try:
        return kill_drone_from_game_master(namespace, drone_model_base_name, drone_models, logger, world_name, gz_node, non_blocking=False)
    except Exception as e:
        if logger:
            logger.error(f"Error in kill drone thread for {namespace}: {e}")
        else:
            print(f"Error in kill drone thread for {namespace}: {e}")
        return False

def kill_drone_from_game_master(namespace, drone_model_base_name, drone_models={}, logger=None, world_name="game_world", gz_node=None, non_blocking=False):
    """
    Remove a drone from the simulation using a sequential process:
    1. Try to remove the Gazebo model
    2. If that fails, try running shutdown_drone
    3. Kill ROS2 nodes with the drone's namespace
    4. For drones other than drone 0, kill the drone's PX4 processes
    
    When non_blocking=True (default), this function starts the sequence in a background thread
    and returns immediately to avoid holding up the game master.
    
    Args:
        namespace (str): The namespace of the drone to remove (e.g., '/px4_1')
        drone_model_base_name (str): Base name pattern for drone models (e.g., 'x500_lidar_front')
        drone_models (dict): Optional dictionary mapping namespaces to model IDs (if already known)
        logger: Optional logger object to receive log messages
        world_name (str): Name of the Gazebo world
        gz_node: Optional existing Gazebo transport node to use
        non_blocking (bool): If True, don't wait for operations to complete
    
    Returns:
        bool: True if the kill sequence was successfully initiated
    """
    # Helper function for logging
    def log_info(msg):
        if logger:
            logger.info(msg)
        else:
            print(msg)
            
    def log_warn(msg):
        if logger:
            logger.warn(msg)
        else:
            print(f"WARNING: {msg}")
            
    def log_error(msg):
        if logger:
            logger.error(msg)
        else:
            print(f"ERROR: {msg}")
    
    if '/px4_' not in namespace:
        log_warn(f"{namespace} is not a drone, cannot kill")
        return False
    
    # Extract instance number from namespace
    instance_number = int(namespace.split('_')[-1])
    
    # Find the model ID if we didn't already have it
    if namespace in drone_models and drone_models[namespace]:
        model_id = drone_models[namespace]
    else:
        model_name = f"{drone_model_base_name}_{instance_number}"
        model_id = get_model_id(model_name, logger, world_name, gz_node)
    
    # Define the sequential kill process
    def sequential_kill_process():
        try:
            # Step 1: Try to remove the model from Gazebo
            model_removed = False
            if model_id:
                log_info(f"Attempting to remove model with ID {model_id} from Gazebo")
                # Use an event to track if removal failed
                fail_event = threading.Event()
                remove_model(model_id, fail_event, logger, world_name, gz_node)
                model_removed = not fail_event.is_set()
                log_info(f"Model removal {'succeeded' if model_removed else 'failed'} for model with ID {model_id}")
            else:
                log_warn(f"No model ID found for {namespace}, skipping Gazebo model removal")
            
            # Step 2: If model removal failed, try shutdown_drone
            cmd_remove = "gz model --list"
            result = subprocess.run(
                cmd_remove,
                shell=True, 
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
                )
            model = f"{drone_model_base_name}_{instance_number}"

            if model in result.stdout and instance_number != 0:
                try:
                    shutdown_drone(instance_number, logger)
                    log_info(f"Shutdown of drone {instance_number} completed")
                except Exception as e:
                    log_error(f"Error shutting down drone {instance_number}: {e}")
            
            # Step 3: Kill ROS2 nodes with the drone's namespace
            try:
                subprocess.run(f'pgrep -f "\-r __ns:={namespace}" | xargs kill -9', shell=True, check=False)
                log_info(f"ROS2 nodes for {namespace} killed")
            except Exception as e:
                log_error(f"Error killing ROS2 nodes for {namespace}: {e}")
            
            # Step 4: For drones other than drone 0, kill the drone's PX4 processes
            if instance_number != 0:
                try:
                    # Kill the px4 instance
                    subprocess.run(f'pkill -f "px4 -i {instance_number}"', shell=True, check=False)
                    # Kill any other px4 related processes
                    subprocess.run(f'pkill -f "px4_{instance_number}"', shell=True, check=False)
                    log_info(f"PX4 processes for drone {instance_number} killed")
                except Exception as e:
                    log_error(f"Error killing PX4 processes for drone {instance_number}: {e}")
            
            log_info(f"Kill sequence for {namespace} completed")
            
        except Exception as e:
            log_error(f"Error in sequential kill process for {namespace}: {e}")
    
    # Start the kill process
    if non_blocking:
        # Start in a separate thread and return immediately
        log_info(f"Started non-blocking kill sequence for {namespace}")
        kill_thread = threading.Thread(
            target=sequential_kill_process,
            daemon=True  # Make thread daemon so it doesn't block program exit
        )
        kill_thread.start()
        return True
    else:
        # Run the process in the current thread
        sequential_kill_process()
        return True

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

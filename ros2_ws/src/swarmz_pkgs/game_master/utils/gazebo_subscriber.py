from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import gz.msgs10.empty_pb2 as gz_empty
import gz.msgs10.scene_pb2 as gz_scene
import tf_transformations
import numpy as np
import time 
import threading
import logging

class GazeboPosesTracker(Node):
    def __init__(self, robot_names, px4_prefix="x500_lidar_front", flag_ship_prefix="flag_ship", world_name="game_world_water", logger=None):
        super().__init__()
        # Create a Gazebo transport node
        self.world_name = world_name
        self.topic_dynamic_pose = "/world/"+world_name+"/dynamic_pose/info"
        self.model_names = []
        self.px4_prefix = px4_prefix
        self.flag_ship_prefix = flag_ship_prefix
        
        # Set the logger 
        if logger:
            self.logger = logger
        else:
            self.logger = logging.getLogger(__name__)
            self.logger.setLevel(logging.INFO)
            console_handler = logging.StreamHandler()
            formatter = logging.Formatter('[%(levelname)s] %(message)s')
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

        # Create a mapping from input names to Gazebo names
        self.name_mapping = {}
        for name in robot_names:
            if not name:
                continue
            if name.startswith("/px4"):
                prefix = self.px4_prefix
                suffix = name.split('_')[1]
            elif name.startswith("/flag_ship"):
                prefix = self.flag_ship_prefix
                suffix = name.split('_')[2]
                # Add gun link
                prefix_gun = "flag_ship_gun"
                self.model_names.append(f"{prefix_gun}_{suffix}")
                self.name_mapping[f"/{prefix_gun}_{suffix}"] = f"{prefix_gun}_{suffix}"
                # Add gr link
                prefix_gr = "flag_ship_gr"
                self.model_names.append(f"{prefix_gr}_{suffix}")
                self.name_mapping[f"/{prefix_gr}_{suffix}"] = f"{prefix_gr}_{suffix}"
                # Add the relative cannon  
                prefix_cannon = "relative_cannon"
                self.model_names.append(f"{prefix_cannon}_{suffix}")
                self.name_mapping[f"/{prefix_cannon}_{suffix}"] = f"{prefix_cannon}_{suffix}"
                # Add the global cannon  
                prefix_cannon = "global_cannon"
                self.model_names.append(f"{prefix_cannon}_{suffix}")
                self.name_mapping[f"/{prefix_cannon}_{suffix}"] = f"{prefix_cannon}_{suffix}"
            else:
                raise ValueError(f"Unknown prefix for model name {name}")
            self.name_mapping[name] = f"{prefix}_{suffix}"
            self.model_names.append(f"{prefix}_{suffix}")
        
        # Create dict to store id and model name for each namespace         
        self.model_id = {
            name.lstrip('/'): {"id": None, "name": None}
            for name in self.name_mapping 
            if not "cannon" in name} 
        
        self.dynamic_poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.model_names}

        # Create link for the service
        self.service_pose = "/world/"+self.world_name+"/scene/info"
        self.link_name = ["gun", "gr"]
        
        for i in range(1, len(self.model_names) + 1):
            setattr(self, f"pitch_id{i}", 0.0) 
            setattr(self, f"yaw_id{i}", 0.0)
            setattr(self, f"pitch_{i}", 0.0)
            setattr(self, f"yaw_{i}", 0.0)
        
        # Get the IDs of the links gun and gr of the flag_ship 
        # Initialize model IDs for all robots
        for robot_name in robot_names:
            if robot_name:
                # Strip the leading slash if present
                clean_name = robot_name.lstrip('/')
                # Only call get_model_id for base model names, not derived ones like gun/gr
                if not any(keyword in clean_name for keyword in ["cannon", "gun", "gr"]):
                    self.get_model_id(clean_name)

        # Log of the models ID and models names detected
        # self.logger.info(f"[gazebo_subscriber] : Detected models ID: {self.model_id}")
        excluded_keywords = ["cannon", "gun", "gr"]
        self.detected_models = [
            name for name in self.model_names
            if not any(keyword in name for keyword in excluded_keywords)
        ]
        self.logger.info(f"[gazebo_subscriber] : Detected models : {self.detected_models}")

        # timmer to update the cannon relative pose 
        self.start_timmer = time.time()
        self.exec_count = 0

        # Subscribe to topics
        try:
            self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
        except Exception as e:
            self.logger.error(f"[gazebo_subscriber] : Error subscribing to topic [{self.topic_dynamic_pose}]: {e}")

    def get_model_id(self, model_name):
        """
        Find the IDs of a specific model of the simulation world by their names using Gazebo Transport API.
        :param model_name: A name of robot namespace names (e.g., ["flag_ship_1", "flag_ship_2", "px4_0", "px4_1"]).
        :return: A dictionary mapping each robot namespace to a sub-dictionary containing model ID and name.
        Example: {"px4_0": {"id": 123, "name": "x500_lidar_front_0"}}
        """
        req = gz_empty.Empty()
        req_byte = req.SerializeToString()
        timeout = 2000  # ms
        try:
            success, response_bytes = self.request_raw(
                self.service_pose,
                req_byte,
                "gz.msgs.Empty",
                "gz.msgs.Scene",
                timeout
            )
            if success:
                scene = gz_scene.Scene()
                scene.ParseFromString(response_bytes)

                for model in scene.model:
                    if model_name.startswith("flag_ship"):
                        id = int(model_name.split('_')[2])

                    if model.name == self.name_mapping[f"/{model_name}"]:
                        self.model_id[model_name]["id"] = model.id
                        self.model_id[model_name]["name"] = model.name

                        for link in model.link:
                            if link.name == self.link_name[0]:
                                setattr(self, f"pitch_id{id}", link.id)
                                self.model_id[f"flag_ship_{link.name}_{id}"]["id"] = link.id
                                self.model_id[f"flag_ship_{link.name}_{id}"]["name"] = link.name

                            elif link.name == self.link_name[1]:
                                setattr(self, f"yaw_id{id}", link.id)
                                self.model_id[f"flag_ship_{link.name}_{id}"]["id"] = link.id
                                self.model_id[f"flag_ship_{link.name}_{id}"]["name"] = link.name

                return self.model_id[model_name]["id"]
            else:
                self.logger.warning(f"[gazebo_subscriber] : Failed to call the service : {response_bytes}")
        except Exception as e:
            self.logger.error(f"[gazebo_subscriber] : Error when calling service : {e}")
    
    def get_cannon_rpy(self):
        """
        Get the orientation of the cannon.
        :param robot: The name of the robot.
        :return: the values of pitch and yaw for the two boats .
        """
        return self.pitch_1, self.pitch_2, self.yaw_1, self.yaw_2 
    
    def get_pose(self, robot):
        """
        Get the pose of the specified model.
        :param robot: The name of the robot.
        :param use_dynamic: If True, use the dynamic pose; otherwise, use the regular pose.
        :return: Dictionary containing the pose of the model.
        """
        if self.name_mapping[robot] not in self.dynamic_poses:
            raise ValueError(f"Model name {self.name_mapping[robot]} not found.")
        return self.dynamic_poses[self.name_mapping[robot]]

    def get_robot_position(self, robot):
        """
        Get the position of the robot with the given namespace.
        :param robot: The namespace of the robot
        :return: Tuple (x, y, z) representing the robot's position
        """
        if not robot:
            raise ValueError("Namespace cannot be empty")
        pose = self.get_pose(robot)
        return (pose['position']['x'], pose['position']['y'], pose['position']['z'])

    def dynamic_pose_cb(self, msg):
        # Update dynamic poses for each model 
        start_all = time.time()
        for pose in msg.pose:
            model_name = pose.name
            if model_name in self.dynamic_poses:
                self.dynamic_poses[model_name]["position"]["x"] = pose.position.x
                self.dynamic_poses[model_name]["position"]["y"] = pose.position.y
                self.dynamic_poses[model_name]["position"]["z"] = pose.position.z
                self.dynamic_poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_poses[model_name]["orientation"]["w"] = pose.orientation.w
            
            # Update the gun and gr pose of both boat
            elif model_name in self.link_name:
                # Case when the service fail to get the IDs
                if self.pitch_id1 == 0.0 or self.pitch_id2 == 0.0 or self.yaw_id1 == 0.0 or self.yaw_id2 == 0.0:
                    self.get_model_id("flag_ship_1")
                    self.get_model_id("flag_ship_2")
                    self.logger.error("[gazebo_subscriber] : Error gz service failed to get the IDs !")
                    return 
                
                if pose.id==self.pitch_id1 or pose.id==self.yaw_id1:
                    model_name = f"flag_ship_{model_name}_{1}"
                elif pose.id==self.pitch_id2 or pose.id==self.yaw_id2:
                    model_name = f"flag_ship_{model_name}_{2}"

                self.dynamic_poses[model_name]["position"]["x"] = pose.position.x
                self.dynamic_poses[model_name]["position"]["y"] = pose.position.y
                self.dynamic_poses[model_name]["position"]["z"] = pose.position.z
                self.dynamic_poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_poses[model_name]["orientation"]["w"] = pose.orientation.w

        # Call the method to calculate the relative pose of the cannon just times per second to not impact the publish rate of the gz topic 
        current_time = time.time()
        if current_time - self.start_timmer < 1.0 and self.exec_count < 2:
            self.exec_count +=1 
            frames_names_1 = ["flag_ship_gr_1", "flag_ship_gun_1"]
            frames_names_2 = ["flag_ship_gr_2", "flag_ship_gun_2"] 
            t1 = threading.Thread(
                target=self.calculate_frame_pose,
                args=(frames_names_1, 1)
            )
            t2 = threading.Thread(
                target=self.calculate_frame_pose,
                args=(frames_names_2, 2)
            )

            t1.start()
            t2.start()
        elif current_time - self.start_timmer >= 1.0:
            # Reinitialize the counter and start_timmer every second
            self.start_timmer = current_time
            self.exec_count = 0
    
    # Call all the function to update the cannon global pose
    def get_global_cannon_pose(self, robot):
        id = robot.split('_')[2]
        frames_names = [f"flag_ship_{id}", f"flag_ship_gr_{id}", f"flag_ship_gun_{id}"]
        return self.calculate_frame_pose(frames_names, id)

    # Calculate the transformation of the frames and return their pose 
    def calculate_frame_pose(self, frame_names, id):
        # Initialize the global transformation matrix as an identity matrix
        global_matrix = np.eye(4)

        # Iterate over the list of frame names
        for frame_name in frame_names:
            position, orientation = self.get_pose_and_orientation(frame_name)
            # Get the transformation matrix for the current frame
            frame_matrix = self.get_transformation_matrix(position, orientation)
            # Concatenate the current frame's matrix to the global matrix
            global_matrix = np.dot(global_matrix, frame_matrix)

        # Extract global position and orientation (global_position is np_array type while global_orientation is tuple type)
        global_position = tf_transformations.translation_from_matrix(global_matrix)
        global_orientation = tf_transformations.quaternion_from_matrix(global_matrix)

        cannon = f"global_cannon_{id}"  if len(frame_names) == 3  else f"relative_cannon_{id}"
        self.dynamic_poses[cannon]["position"]["x"] = global_position[0]
        self.dynamic_poses[cannon]["position"]["y"] = global_position[1]
        self.dynamic_poses[cannon]["position"]["z"] = global_position[2]
        self.dynamic_poses[cannon]["orientation"]["x"] = global_orientation[0]
        self.dynamic_poses[cannon]["orientation"]["y"] = global_orientation[1]
        self.dynamic_poses[cannon]["orientation"]["z"] = global_orientation[2]
        self.dynamic_poses[cannon]["orientation"]["w"] = global_orientation[3]

        return self.dynamic_poses[cannon]
        
    # Get the the position and orientation of a specific frame and return in np.array type
    def get_pose_and_orientation(self, model_name, local=False):
        position = np.array([
            self.dynamic_poses[model_name]["position"]["x"],
            self.dynamic_poses[model_name]["position"]["y"],
            self.dynamic_poses[model_name]["position"]["z"]
        ])
        orientation = np.array([
            self.dynamic_poses[model_name]["orientation"]["x"],
            self.dynamic_poses[model_name]["orientation"]["y"],
            self.dynamic_poses[model_name]["orientation"]["z"],
            self.dynamic_poses[model_name]["orientation"]["w"]
        ])

        if "gun" in model_name and local==False:
            # The initial pose/orientation of the "gun" link have an offset of -90° in pitch compared with the flag_ship base link
            offset_quat = tf_transformations.quaternion_from_euler(0.0, -np.pi/2, 0.0)  # roll=0, pitch=-90°, yaw=0
            orientation = tf_transformations.quaternion_multiply(offset_quat, orientation)
    
        return position, orientation
    
    # Convert the transformation (position and orientation) in matrix
    def get_transformation_matrix(self, position, orientation):
        # Create translation matrix
        translation_matrix = tf_transformations.translation_matrix(position)
        # Create rotation matrix from quaternion
        rotation_matrix = tf_transformations.quaternion_matrix(orientation)
        # Combine translation and rotation into a single matrix
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        return transformation_matrix
    
    def print_pose_deltas(self):
        print("\n--- Current Poses ---")
        for model_name, pose in self.poses.items():
            print(f"Model: {model_name}")
            print(f"Position: {pose['position']}")
            print(f"Orientation: {pose['orientation']}")

def main():
    robot_names = ['/flag_ship_1', '/flag_ship_2', "/px4_1", "/px4_2"]
    gz = GazeboPosesTracker(robot_names)
    try:
        for robot in robot_names:
            pose = gz.get_pose(robot)
            gz.logger.info(f"[gazebo_subscriber] : {robot}")        
            gz.logger.info(f"[gazebo_subscriber] : {pose}") # The first time this poses are not updtate due to the delay of getting the data from the gz topic
    except KeyboardInterrupt:
        gz.logger.info("[gazebo_subscriber] : Shutting down...")

if __name__ == "__main__":
    main()


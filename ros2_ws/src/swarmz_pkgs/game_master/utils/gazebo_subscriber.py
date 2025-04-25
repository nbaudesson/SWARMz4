from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import threading
import copy
from scipy.spatial.transform import Rotation as R # TODO: Find alternative with numpy or transform3d
import gz.msgs10.empty_pb2 as gz_empty
import gz.msgs10.scene_pb2 as gz_scene

class GazeboPosesTracker(Node):

    def __init__(self, robot_names, px4_prefix="x500_lidar_front", flag_ship_prefix="flag_ship", world_name="swarmz_world_2"):
        super().__init__()

        # Create a Gazebo transport node
        self.topic_dynamic_pose = "/world/"+world_name+"/dynamic_pose/info"
        self.topic_pose = "/world/"+world_name+"/pose/info"
        self.model_names = []
        self.px4_prefix = px4_prefix
        self.flag_ship_prefix = flag_ship_prefix
        self.use_dynamic = False

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
            else:
                raise ValueError(f"Unknown prefix for model name {name}")
            self.name_mapping[name] = f"{prefix}_{suffix}"
            self.model_names.append(f"{prefix}_{suffix}")
        
        # print(f"Name mapping: {self.name_mapping}")
        # print(f"robot names: {robot_names}")
        # print(f"model names: {self.model_names}")
        
        self.poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.model_names}
        self.dynamic_poses = copy.deepcopy(self.poses)

        # Ajout
        # Create link for the service
        self.service_pose = "/world/"+world_name+"/scene/info"
        self.link_name = ["gun", "gr"]
        self.pitch_id1 = self.pitch_id2 = self.yaw_id1 = self.yaw_id2 = 0.0
        self.pitch_1 = self.pitch_2 = self.yaw_1 = self.yaw_2 = 0.0
        self.link_poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.link_name}
        self.dynamic_link_poses = copy.deepcopy(self.link_poses)
 
        for i in range(1, len(self.model_names) + 1):
            setattr(self, f"pitch_id{i}", None)  # ou une valeur par défaut
            setattr(self, f"yaw_id{i}", None)
        self.get_id()
        
        if self.use_dynamic:
            # Subscribe to topics
            try:
                self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
                # print(f"Subscribed to topic [{self.topic_dynamic_pose}]")
            except Exception as e:
                print(f"Error subscribing to topic [{self.topic_dynamic_pose}]: {e}")
        else:
            try:
                self.subscribe(Pose_V, self.topic_pose, self.pose_cb)
                # print(f"Subscribed to topic [{self.topic_pose}]")
            except Exception as e:
                print(f"Error subscribing to topic [{self.topic_pose}]: {e}")

    def get_id(self):
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
                
                id = 1
                for model in scene.model:
                    model_name = model.name
                    if model_name in self.model_names:
                        print("Modèle trouvé :", model_name)
                        for link in model.link:
                            if link.name == self.link_name[0]:
                                print(f"🔍 ID du link 'gun' : {link.id}")
                                setattr(self, f"pitch_id{id}", link.id)
                            elif link.name == self.link_name[1]:
                                print(f"🔍 ID du link 'gr' : {link.id}")
                                setattr(self, f"yaw_id{id}", link.id)
                            elif id <len(self.model_names) and getattr(self, f"pitch_id{id}")!=None and getattr(self, f"yaw_id{id}")!=None:
                                id += 1
                            
                                # print(f"Position : x={link.pose.position.x}, y={link.pose.position.y}, z={link.pose.position.z}")
                                # print(f"Orientation : x={link.pose.orientation.x}, y={link.pose.orientation.y}, z={link.pose.orientation.z}, w={link.pose.orientation.w}")
                print(self.pitch_id1, self.pitch_id2, self.yaw_id1, self.yaw_id2)
            else:
                print("Échec de l'appel au service.", response_bytes)
 
        except Exception as e:
            print(f"Erreur lors de l'appel du service : {e}")
 
    
    def get_cannon_rpy(self):
        """
        Get the orientation of the cannon.
        :param robot: The name of the robot.
        :return: the values of pitch and yaw for the two boats .
        """
        return self.pitch_1, self.pitch_2, self.yaw_1, self.yaw_2  
    
    def get_pose(self, robot, use_dynamic=False):
        """
        Get the pose of the specified model.
        :param robot: The name of the robot.
        :param use_dynamic: If True, use the dynamic pose; otherwise, use the regular pose.
        :return: Dictionary containing the pose of the model.
        """
        # print(f"poses =  [{self.poses}]")
        if self.name_mapping[robot] not in self.poses:
            raise ValueError(f"Model name {self.name_mapping[robot]} not found.")
        return self.dynamic_poses[self.name_mapping[robot]] if use_dynamic else self.poses[self.name_mapping[robot]]

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
        for pose in msg.pose:
            model_name = pose.name
            # print('model name :', model_name)
            if model_name in self.dynamic_poses:
                self.dynamic_poses[model_name]["position"]["x"] = pose.position.x
                self.dynamic_poses[model_name]["position"]["y"] = pose.position.y
                self.dynamic_poses[model_name]["position"]["z"] = pose.position.z
                self.dynamic_poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_poses[model_name]["orientation"]["w"] = pose.orientation.w
            
            elif model_name in self.link_name:
                self.dynamic_link_poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_link_poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_link_poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_link_poses[model_name]["orientation"]["w"] = pose.orientation.w
                q = pose.orientation
                quaternion = [q.x, q.y, q.z, q.w]
                rotation = R.from_quat(quaternion)  # Convertit le quaternion en un objet de rotation
                roll, pitch, yaw = rotation.as_euler('xyz', degrees=True)
                if pose.id==self.pitch_id1:
                    self.pitch_1 = pitch * 6.28 / 360
                elif pose.id==self.pitch_id2:
                    self.pitch_2 = pitch * 6.28 / 360
                elif pose.id==self.yaw_id1:
                    self.yaw_1= yaw * 6.28 / 360
                elif pose.id==self.yaw_id2:
                    self.yaw_2= yaw * 6.28 / 360
 
                # print('pitch1: ', self.pitch_1, 'pitch2 :', self.pitch_2, 'yaw1 :', self.yaw_1, 'yaw2 :', self.yaw_2)
 
        # print(f"Updated dynamic poses = [{self.dynamic_poses}]")

    def pose_cb(self, msg):
        # Update poses for each model
        for pose in msg.pose:
            model_name = pose.name
            if model_name in self.poses:
                self.poses[model_name]["position"]["x"] = pose.position.x
                self.poses[model_name]["position"]["y"] = pose.position.y
                self.poses[model_name]["position"]["z"] = pose.position.z
                self.poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.poses[model_name]["orientation"]["w"] = pose.orientation.w
        # print(f"Updated poses = [{self.poses}]")

    def print_pose_deltas(self):
        print("\n--- Current Poses ---")
        for model_name, pose in self.poses.items():
            print(f"Model: {model_name}")
            print(f"Position: {pose['position']}")
            print(f"Orientation: {pose['orientation']}")

def main():
    robot_names = ["/px4_1", "/px4_2"]
    gazebo_poses = GazeboPosesTracker(robot_names)
    try:
        def print_pose():
            for robot in robot_names:
                pose = gazebo_poses.get_pose(robot)
                print(f"\nCurrent Pose for {robot}:", pose)
            threading.Timer(1.0, print_pose).start()

        print_pose()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()
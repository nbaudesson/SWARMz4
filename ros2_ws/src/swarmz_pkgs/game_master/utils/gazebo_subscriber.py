from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import threading
import copy
import rclpy.parameter

class GazeboPosesTracker(Node):

    def __init__(self, model_names, px4_prefix="x500", flag_ship_prefix="y600"):
        super().__init__()
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Create a Gazebo transport node
        self.topic_dynamic_pose = "/world/default/dynamic_pose/info"
        self.topic_pose = "/world/default/pose/info"
        self.model_names = model_names
        self.px4_prefix = px4_prefix
        self.flag_ship_prefix = flag_ship_prefix

        # Create a mapping from input names to Gazebo names
        self.name_mapping = {}
        for name in model_names:
            if name.startswith("px4"):
                prefix = self.px4_prefix
            elif name.startswith("flag_ship"):
                prefix = self.flag_ship_prefix
            else:
                raise ValueError(f"Unknown prefix for model name {name}")
            suffix = name.split('_')[1]
            self.name_mapping[name] = f"{prefix}_{suffix}"
        
        self.poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in model_names}
        self.dynamic_poses = copy.deepcopy(self.poses)

        # Subscribe to topics
        try:
            self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
            print(f"Subscribed to topic [{self.topic_dynamic_pose}]")
        except Exception as e:
            print(f"Error subscribing to topic [{self.topic_dynamic_pose}]: {e}")
        
        try:
            self.subscribe(Pose_V, self.topic_pose, self.pose_cb)
            print(f"Subscribed to topic [{self.topic_pose}]")
        except Exception as e:
            print(f"Error subscribing to topic [{self.topic_pose}]: {e}")

    def get_pose(self, model_name, use_dynamic=False):
        """
        Get the pose of the specified model.
        :param model_name: The name of the model.
        :param use_dynamic: If True, use the dynamic pose; otherwise, use the regular pose.
        :return: Dictionary containing the pose of the model.
        """
        if model_name not in self.poses:
            raise ValueError(f"Model name {model_name} not found.")
        return self.dynamic_poses[model_name] if use_dynamic else self.poses[model_name]

    def dynamic_pose_cb(self, msg):
        # Update dynamic poses for each model
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

    def print_pose_deltas(self):
        print("\n--- Current Poses ---")
        for model_name, pose in self.poses.items():
            print(f"Model: {model_name}")
            print(f"Position: {pose['position']}")
            print(f"Orientation: {pose['orientation']}")

def main():
    model_names = ["x500_lidar_front_1", "x500_lidar_front_2"]
    gazebo_poses = GazeboPosesTracker(model_names)
    try:
        def print_pose():
            for model_name in model_names:
                pose = gazebo_poses.get_pose(model_name)
                print(f"\nCurrent Pose for {model_name}:", pose)
            threading.Timer(1.0, print_pose).start()

        print_pose()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()
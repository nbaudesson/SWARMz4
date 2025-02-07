from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import threading
import copy

class GazeboPosesTracker(Node):

    def __init__(self, robot_names, px4_prefix="x500_lidar_front", flag_ship_prefix="flag_ship"):
        super().__init__()

        # Create a Gazebo transport node
        self.topic_dynamic_pose = "/world/default/dynamic_pose/info"
        self.topic_pose = "/world/default/pose/info"
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
            elif name.startswith("/flag_ship"):
                prefix = self.flag_ship_prefix
            else:
                raise ValueError(f"Unknown prefix for model name {name}")
            suffix = name.split('_')[1]
            self.name_mapping[name] = f"{prefix}_{suffix}"
            self.model_names.append(f"{prefix}_{suffix}")
        
        # print(f"Name mapping: {self.name_mapping}")
        # print(f"robot names: {robot_names}")
        # print(f"model names: {self.model_names}")
        
        self.poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.model_names}
        self.dynamic_poses = copy.deepcopy(self.poses)

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
            if model_name in self.dynamic_poses:
                self.dynamic_poses[model_name]["position"]["x"] = pose.position.x
                self.dynamic_poses[model_name]["position"]["y"] = pose.position.y
                self.dynamic_poses[model_name]["position"]["z"] = pose.position.z
                self.dynamic_poses[model_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_poses[model_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_poses[model_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_poses[model_name]["orientation"]["w"] = pose.orientation.w
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
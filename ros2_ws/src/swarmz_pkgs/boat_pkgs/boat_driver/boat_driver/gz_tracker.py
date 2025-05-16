from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import gz.msgs10.empty_pb2 as gz_empty
import gz.msgs10.scene_pb2 as gz_scene
import tf_transformations
 
class GazeboPosesTracker(Node):
 
    def __init__(self, robot_names, flag_ship_prefix="flag_ship", world_name="swarmz_world_2"):
        super().__init__()
 
        # Create a Gazebo transport node
        self.world_name=world_name
        self.topic_dynamic_pose = "/world/"+self.world_name+"/dynamic_pose/info"
        self.flag_ship_prefix = flag_ship_prefix
        self.model_names = []
 
        # Create a mapping from input names to Gazebo names
        for name in robot_names:
            if not name:
                continue
            if name.startswith("/flag_ship"):
                prefix = self.flag_ship_prefix
                suffix = name.split('_')[2]
            else:
                raise ValueError(f"Unknown prefix for model name {name}")
            self.model_names.append(f"{prefix}_{suffix}")
 
        # Create link for the service
        self.service_pose = "/world/"+self.world_name+"/scene/info"
        self.link_name = ["gun", "gr"]
        self.dynamic_link_poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.link_name}
 
        for i in range(1, len(self.model_names) + 1):
            print("indice i :", i)
            setattr(self, f"pitch_id{i}", 0.0)  
            setattr(self, f"yaw_id{i}", 0.0)
            setattr(self, f"pitch_{i}", 0.0)  
            setattr(self, f"yaw_{i}", 0.0)

        self.get_id()
 
        # Subscribe to topics
        try:
            self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
            print(f"Subscribed to topic [{self.topic_dynamic_pose}]")
        except Exception as e:
            print(f"Error subscribing to topic [{self.topic_dynamic_pose}]: {e}")
 
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
                        for link in model.link:
                            if link.name == self.link_name[0]:
                                print(f"🔍 ID du link 'gun' : {link.id}")
                                setattr(self, f"pitch_id{id}", link.id)
                            elif link.name == self.link_name[1]:
                                print(f"🔍 ID du link 'gr' : {link.id}")
                                setattr(self, f"yaw_id{id}", link.id)
                            elif id <len(self.model_names) and str(getattr(self, f"pitch_id{id}")) != "0.0" and str(getattr(self, f"yaw_id{id}"))!= "0.0":
                                id += 1
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
 
    def dynamic_pose_cb(self, msg):
        # Update dynamic poses for each link
        for pose in msg.pose:
            link_name = pose.name
            if link_name in self.link_name:
                self.dynamic_link_poses[link_name]["orientation"]["x"] = pose.orientation.x
                self.dynamic_link_poses[link_name]["orientation"]["y"] = pose.orientation.y
                self.dynamic_link_poses[link_name]["orientation"]["z"] = pose.orientation.z
                self.dynamic_link_poses[link_name]["orientation"]["w"] = pose.orientation.w
                q = pose.orientation
                quaternion = [q.x, q.y, q.z, q.w]
                roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

                if pose.id==self.pitch_id1:
                    self.pitch_1 = pitch 
                elif pose.id==self.pitch_id2:
                    self.pitch_2 = pitch 
                elif pose.id==self.yaw_id1:
                    self.yaw_1= yaw 
                elif pose.id==self.yaw_id2:
                    self.yaw_2= yaw 
 
def main():
    robot_names = ["/flag_ship_1", "/flag_ship_2"] 
    gazebo_poses = GazeboPosesTracker(robot_names)
    try:
        for robot in robot_names:
            pose = gazebo_poses.get_cannon_rpy()
            print(robot)
            print(pose)
    except KeyboardInterrupt:
        print("Shutting down...")
 
if __name__ == "__main__":
    main()
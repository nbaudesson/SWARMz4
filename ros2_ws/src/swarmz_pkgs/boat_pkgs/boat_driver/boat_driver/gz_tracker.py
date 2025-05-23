from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import gz.msgs10.empty_pb2 as gz_empty
import gz.msgs10.scene_pb2 as gz_scene
import tf_transformations
import logging 
 
class GazeboPosesTracker(Node):
 
    def __init__(self, robot_name, world_name="game_world_water", logger=None):
        super().__init__()
 
        # Create a Gazebo transport node
        self.world_name=world_name
        self.topic_dynamic_pose = "/world/"+self.world_name+"/dynamic_pose/info"
        self.model_name = robot_name
 
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

        # Create link for the service
        self.service_pose = "/world/"+self.world_name+"/scene/info"
        self.link_name = ["gun", "gr"]
        self.dynamic_link_poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.link_name}
 
        self.pitch_id = 0.0
        self.yaw_id = 0.0
        self.yaw = 0.0
        self.pitch = 0.0

        self.get_id()
 
        # Subscribe to topics
        try:
            self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
        except Exception as e:
            self.logger.error(f"[gz_tracker] : Error subscribing to topic [{self.topic_dynamic_pose}]: {e}")
 
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

                for model in scene.model:
                    model_name = model.name
                    if f'/{model_name}' in self.model_name:
                        for link in model.link:
                            if link.name == self.link_name[0]:
                                self.logger.info(f"[gz_tracker] : ID of link 'gun' : {link.id}")
                                self.pitch_id = link.id
                            elif link.name == self.link_name[1]:
                                self.logger.info(f"[gz_tracker] : ID of link 'gr' : {link.id}")
                                self.yaw_id = link.id

            else:
                self.logger.warning(f"[gz_tracker] : Service call failed : {response_bytes}")
 
        except Exception as e:
            self.logger.error(f"[gz_tracker] : Error during service call : {e}")
 
    
    def get_cannon_rpy(self):
        """
        Get the orientation of the cannon.
        :param robot: The name of the robot.
        :return: the values of pitch and yaw for the two boats .
        """
        return self.pitch, self.yaw 
 
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

                if pose.id==self.pitch_id:
                    self.pitch = pitch 
                elif pose.id==self.yaw_id:
                    self.yaw= yaw 

 
def main():
    robot_names = ["/flag_ship_1"] 
    gazebo_poses = GazeboPosesTracker(robot_names)
    try:
        
        pose = gazebo_poses.get_cannon_rpy()
        gazebo_poses.logger.info(f"[gz_tracker] : pose")
        print("DEBUG")

    except KeyboardInterrupt:
        gazebo_poses.logger.info("[gz_tracker] : Shutting down...")
 
if __name__ == "__main__":
    main()
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
import threading
import copy
import time
import sys
import subprocess

class GazeboPosesTracker(Node):

    def __init__(self, robot_names, px4_prefix="x500_lidar_front", flag_ship_prefix="flag_ship", world_name="default"):
        super().__init__()

        print(f"Initializing GazeboPosesTracker with world_name: {world_name}")
        print(f"Robot names: {robot_names}")

        # Add connection verification flags
        self.connection_verified = False
        self.connection_check_complete = False
        
        # Increase timeout for Gazebo commands
        self.command_timeout = 10.0  # Increased from 2 seconds to 10 seconds
        
        # Create a Gazebo transport node - support both topic formats
        self.topic_formats = [
            # Standard format with world name
            {
                "dynamic_pose": f"/world/{world_name}/dynamic_pose/info",
                "pose": f"/world/{world_name}/pose/info"
            },
            # Alternative format without world name
            {
                "dynamic_pose": "/pose/info",
                "pose": "/pose/info"
            }
        ]
        
        # Will be set to the working topic format
        self.active_topic_format = None
        self.topic_dynamic_pose = None
        self.topic_pose = None
        
        self.model_names = []
        self.px4_prefix = px4_prefix
        self.flag_ship_prefix = flag_ship_prefix
        self.use_dynamic = False
        self.last_message_time = 0
        self.message_count = 0
        self.world_name = world_name

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
                print(f"Unknown prefix for model name {name}, skipping")
                continue
            suffix = name.split('_')[-1]
            self.name_mapping[name] = f"{prefix}_{suffix}"
            self.model_names.append(f"{prefix}_{suffix}")
        
        print(f"Name mapping: {self.name_mapping}")
        print(f"Model names to track: {self.model_names}")
        
        self.poses = {name: {"position": {"x": None, "y": None, "z": None}, "orientation": {"x": None, "y": None, "z": None, "w": None}} for name in self.model_names}
        self.dynamic_poses = copy.deepcopy(self.poses)

        # Verify Gazebo connection before proceeding
        self.verify_gazebo_connection()

        # Try both pose topics for better reliability
        self._auto_detect_topic_format()
        
        # Start a debug thread
        self._start_debug_thread()

    def verify_gazebo_connection(self):
        """Check if Gazebo is running and accessible"""
        print("Verifying Gazebo connection...")
        
        # First check if we have any Gazebo topics at all
        try:
            print(f"Running 'gz topic -l' with {self.command_timeout}s timeout...")
            result = subprocess.run(['gz', 'topic', '-l'], 
                                   capture_output=True, text=True, 
                                   timeout=self.command_timeout)  # Use increased timeout
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print(f"Gazebo has {len(topics)} topics available")
                
                # As long as we have SOME topics, we'll consider Gazebo running
                if len(topics) > 0:
                    self.connection_verified = True
                    print("✓ Gazebo is running (topics are available)")
                    
                    # Extract potential world names
                    world_topics = [t for t in topics if '/world/' in t]
                    if world_topics:
                        potential_worlds = set()
                        for topic in world_topics:
                            parts = topic.split('/')
                            if len(parts) > 2:
                                potential_worlds.add(parts[2])
                        
                        if potential_worlds:
                            print(f"Detected potential worlds: {potential_worlds}")
                            # If our configured world isn't in the list but others are, 
                            # suggest changing the world name
                            if self.world_name not in potential_worlds:
                                print(f"WARNING: Configured world '{self.world_name}' not found.")
                                print(f"Available worlds: {potential_worlds}")
                                print(f"Consider changing world_name parameter.")
                    else:
                        # No world topics but Gazebo is running
                        print("No world/* topics found, but Gazebo is running")
                        print("Will attempt to use model-based topics instead")
                else:
                    self.connection_verified = False
                    print("✗ No Gazebo topics found. Is Gazebo running correctly?")
            else:
                self.connection_verified = False
                print(f"✗ Failed to list Gazebo topics: {result.stderr}")
        except subprocess.TimeoutExpired:
            print(f"WARNING: 'gz topic -l' timed out after {self.command_timeout} seconds")
            print("This usually means Gazebo is overloaded or running on a slow machine")
            # Consider it running if we can detect the process
            process_running = self._check_gazebo_process()
            if process_running:
                self.connection_verified = True
                print("✓ Gazebo processes are running, continuing with limited functionality")
            else:
                self.connection_verified = False
        except Exception as e:
            self.connection_verified = False
            print(f"✗ Error checking Gazebo topics: {e}")
        
        # Check for running Gazebo processes (alternative verification)
        try:
            result = subprocess.run(['ps', '-ef'], capture_output=True, text=True)
            if 'gz sim' in result.stdout:
                print("✓ Detected 'gz sim' process running")
                self.connection_verified = True
            elif 'gzserver' in result.stdout:
                print("✓ Detected 'gzserver' process running")
                self.connection_verified = True
        except Exception as e:
            print(f"Error checking for Gazebo processes: {e}")
        
        self.connection_check_complete = True
        print(f"Gazebo connection verification {'PASSED' if self.connection_verified else 'FAILED'}")
        return self.connection_verified

    def _check_gazebo_process(self):
        """Check if Gazebo processes are running"""
        try:
            result = subprocess.run(['ps', '-ef'], capture_output=True, text=True)
            return 'gz sim' in result.stdout or 'gzserver' in result.stdout
        except Exception as e:
            print(f"Error checking for Gazebo processes: {e}")
            return False

    def _auto_detect_topic_format(self):
        """Auto-detect which topic format is available in Gazebo"""
        print("Auto-detecting Gazebo topic format...")
        try:
            # Get list of available topics with increased timeout
            print(f"Running 'gz topic -l' with {self.command_timeout}s timeout...")
            result = subprocess.run(['gz', 'topic', '-l'], 
                                   capture_output=True, text=True, 
                                   timeout=self.command_timeout)
            if result.returncode != 0:
                print(f"Failed to list Gazebo topics: {result.stderr}")
                return False
                
            topics = result.stdout.strip().split('\n')
            print(f"Found {len(topics)} Gazebo topics")
            
            # First check if world-based topics exist
            for topic_format in self.topic_formats:
                pose_topic = topic_format["pose"]
                if pose_topic in topics:
                    print(f"Found matching topic format: {pose_topic}")
                    self.active_topic_format = topic_format
                    self.topic_pose = pose_topic
                    self.topic_dynamic_pose = topic_format["dynamic_pose"]
                    self._subscribe_to_topics()
                    return True
            
            # If no direct match, look for pose topics with any pattern
            pose_topics = [t for t in topics if "pose" in t]
            if pose_topics:
                print(f"Found {len(pose_topics)} pose-related topics:")
                for topic in pose_topics[:3]:  # Show first 3
                    print(f"  {topic}")
                
                # Try to find topic containing model information
                model_topics = [t for t in topics if "/model/" in t]
                if model_topics:
                    print(f"Found model-related topics, will use model-based tracking")
                    self._subscribe_to_model_topics(model_topics)
                    return True
                    
            # No suitable topics found
            print("No suitable pose topics found in Gazebo")
            return False
            
        except subprocess.TimeoutExpired:
            print(f"WARNING: 'gz topic -l' timed out after {self.command_timeout} seconds")
            print("Will try direct model subscription as fallback")
            self._setup_direct_model_access()
            return True  # Consider this successful so we don't keep retrying
        except Exception as e:
            print(f"Error detecting topic format: {e}")
            # Try direct model access as fallback
            self._setup_direct_model_access()
            return True  # Consider this successful so we don't keep retrying

    def _setup_direct_model_access(self):
        """Set up direct model access when topics can't be discovered"""
        print("Setting up direct model access...")
        self.use_direct_access = True
        self.topic_pose = None
        
        # We'll create a thread to update model positions via direct API calls
        self._start_direct_model_access_thread()
        
        return True

    def _start_direct_model_access_thread(self):
        """Start a thread to periodically update model positions via direct API calls"""
        def direct_update_loop():
            print("Starting direct model access thread")
            update_count = 0
            while True:
                update_count += 1
                try:
                    # Try to directly get model positions from Gazebo via subprocess
                    for model_name in self.model_names:
                        try:
                            # Use gz model to get position (more reliable when topics timeout)
                            cmd = ['gz', 'model', '-m', model_name, '-p']
                            result = subprocess.run(cmd, 
                                                   capture_output=True, text=True, 
                                                   timeout=self.command_timeout/2)
                            
                            if result.returncode == 0 and result.stdout:
                                # Parse position data - typical output: "Position [m] = 1 2 3"
                                pos_match = result.stdout.strip()
                                if "Position" in pos_match:
                                    pos_parts = pos_match.split('=')[1].strip().split()
                                    if len(pos_parts) >= 3:
                                        x, y, z = float(pos_parts[0]), float(pos_parts[1]), float(pos_parts[2])
                                        # Update our internal state
                                        self.poses[model_name]["position"]["x"] = x
                                        self.poses[model_name]["position"]["y"] = y
                                        self.poses[model_name]["position"]["z"] = z
                                        
                                        # Also update orientation if possible
                                        cmd = ['gz', 'model', '-m', model_name, '-o']
                                        result = subprocess.run(cmd, 
                                                               capture_output=True, text=True,
                                                               timeout=self.command_timeout/2)
                                        if result.returncode == 0 and result.stdout:
                                            ori_match = result.stdout.strip()
                                            if "Orientation" in ori_match:
                                                ori_parts = ori_match.split('=')[1].strip().split()
                                                if len(ori_parts) >= 4:
                                                    qx, qy, qz, qw = float(ori_parts[0]), float(ori_parts[1]), float(ori_parts[2]), float(ori_parts[3])
                                                    self.poses[model_name]["orientation"]["x"] = qx
                                                    self.poses[model_name]["orientation"]["y"] = qy
                                                    self.poses[model_name]["orientation"]["z"] = qz
                                                    self.poses[model_name]["orientation"]["w"] = qw
                                        
                                        # Consider this a valid message
                                        self.message_count += 1
                                        self.last_message_time = time.time()
                        except Exception as model_err:
                            if update_count % 10 == 0:  # Log only occasionally
                                print(f"Error getting position for {model_name}: {model_err}")
                    
                    # Sleep a bit between updates
                    time.sleep(1.0)
                    
                    # Log progress occasionally
                    if update_count % 10 == 0:
                        valid_models = sum(1 for m in self.model_names 
                                          if self.poses[m]["position"]["x"] is not None)
                        print(f"Direct model access update #{update_count}: {valid_models}/{len(self.model_names)} models have valid positions")
                        
                except Exception as e:
                    print(f"Error in direct model access thread: {e}")
                    time.sleep(2.0)  # Longer sleep on error
        
        # Start the thread
        direct_thread = threading.Thread(target=direct_update_loop)
        direct_thread.daemon = True
        direct_thread.start()

    def _subscribe_to_topics(self):
        """Subscribe to Gazebo pose topics with better error handling"""
        success = False
        
        # Skip if we don't have a topic format set
        if not self.topic_pose:
            print("No pose topic set, cannot subscribe")
            return False
            
        # Try dynamic pose topic if needed
        if self.use_dynamic and self.topic_dynamic_pose:
            try:
                print(f"Attempting to subscribe to dynamic pose topic: {self.topic_dynamic_pose}")
                success = self.subscribe(Pose_V, self.topic_dynamic_pose, self.dynamic_pose_cb)
                if success:
                    print(f"Successfully subscribed to {self.topic_dynamic_pose}")
                else:
                    print(f"Failed to subscribe to {self.topic_dynamic_pose}")
            except Exception as e:
                print(f"Error subscribing to {self.topic_dynamic_pose}: {str(e)}")
        
        # Always try regular pose topic
        try:
            print(f"Attempting to subscribe to pose topic: {self.topic_pose}")
            success = self.subscribe(Pose_V, self.topic_pose, self.pose_cb)
            if success:
                print(f"Successfully subscribed to {self.topic_pose}")
            else:
                print(f"Failed to subscribe to {self.topic_pose}")
                self._check_available_topics()
        except Exception as e:
            print(f"Error subscribing to {self.topic_pose}: {str(e)}")
            self._check_available_topics()
            
        return success

    def _check_available_topics(self):
        """Check which Gazebo topics are available"""
        try:
            print("\nChecking available Gazebo topics...")
            # Use increased timeout
            result = subprocess.run(['gz', 'topic', '-l'], 
                                   capture_output=True, text=True,
                                   timeout=self.command_timeout)
            if result.returncode == 0:
                topics = result.stdout.strip().split('\n')
                print(f"Available Gazebo topics ({len(topics)}):")
                world_topics = [t for t in topics if '/world/' in t]
                for topic in world_topics:
                    print(f"  {topic}")
                
                # Try to find valid pose topics for our world
                pose_topics = [t for t in topics if self.world_name in t and 'pose' in t]
                if pose_topics:
                    print(f"\nFound possible pose topics for {self.world_name}:")
                    for topic in pose_topics:
                        print(f"  {topic}")
                    
                    # Try subscribing to the first matching topic
                    if pose_topics:
                        alternate_topic = pose_topics[0]
                        print(f"Attempting to subscribe to alternate topic: {alternate_topic}")
                        try:
                            success = self.subscribe(Pose_V, alternate_topic, self.pose_cb)
                            if success:
                                self.topic_pose = alternate_topic
                                print(f"Successfully subscribed to alternate topic: {alternate_topic}")
                        except Exception as e:
                            print(f"Error subscribing to alternate topic: {str(e)}")
            else:
                print("Failed to list Gazebo topics")
        except subprocess.TimeoutExpired:
            print(f"WARNING: 'gz topic -l' timed out after {self.command_timeout} seconds")
            # Set up direct model access as fallback
            self._setup_direct_model_access()
        except Exception as e:
            print(f"Error checking available topics: {str(e)}")

    def are_poses_valid(self):
        """Check if we have received valid pose data"""
        if self.message_count == 0:
            return False
            
        # Check if we have valid data for at least one model
        for model_name in self.model_names:
            if model_name in self.poses:
                pos = self.poses[model_name]["position"]
                if pos["x"] is not None and pos["y"] is not None and pos["z"] is not None:
                    return True
        return False

    def get_pose(self, robot, use_dynamic=False):
        """
        Get the pose of the specified model.
        :param robot: The name of the robot.
        :param use_dynamic: If True, use the dynamic pose; otherwise, use the regular pose.
        :return: Dictionary containing the pose of the model.
        """
        if robot not in self.name_mapping:
            print(f"Warning: Robot name '{robot}' not found in mapping")
            return {"position": {"x": None, "y": None, "z": None}, 
                    "orientation": {"x": None, "y": None, "z": None, "w": None}}
            
        model_name = self.name_mapping[robot]
        if model_name not in self.poses:
            print(f"Warning: Model name '{model_name}' not found in poses dictionary")
            return {"position": {"x": None, "y": None, "z": None}, 
                    "orientation": {"x": None, "y": None, "z": None, "w": None}}
            
        return self.dynamic_poses[model_name] if use_dynamic else self.poses[model_name]

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
        """Callback for dynamic pose updates"""
        try:
            self.message_count += 1
            self.last_message_time = time.time()
            
            if self.message_count == 1 or self.message_count % 100 == 0:
                print(f"Received dynamic pose message #{self.message_count} with {len(msg.pose)} poses")
            
            # Update dynamic poses for each model
            models_updated = 0
            for pose in msg.pose:
                model_name = pose.name
                if model_name in self.dynamic_poses:
                    models_updated += 1
                    self.dynamic_poses[model_name]["position"]["x"] = pose.position.x
                    self.dynamic_poses[model_name]["position"]["y"] = pose.position.y
                    self.dynamic_poses[model_name]["position"]["z"] = pose.position.z
                    self.dynamic_poses[model_name]["orientation"]["x"] = pose.orientation.x
                    self.dynamic_poses[model_name]["orientation"]["y"] = pose.orientation.y
                    self.dynamic_poses[model_name]["orientation"]["z"] = pose.orientation.z
                    self.dynamic_poses[model_name]["orientation"]["w"] = pose.orientation.w
            
            if self.message_count == 1:
                print(f"First dynamic pose message: updated {models_updated} models")
        except Exception as e:
            print(f"Error in dynamic_pose_cb: {str(e)}")

    def pose_cb(self, msg):
        """Callback for regular pose updates"""
        try:
            self.message_count += 1
            self.last_message_time = time.time()
            
            if self.message_count == 1 or self.message_count % 100 == 0:
                print(f"Received pose message #{self.message_count} with {len(msg.pose)} poses")
            
            # Update poses for each model
            models_updated = 0
            for pose in msg.pose:
                model_name = pose.name
                if model_name in self.poses:
                    models_updated += 1
                    self.poses[model_name]["position"]["x"] = pose.position.x
                    self.poses[model_name]["position"]["y"] = pose.position.y
                    self.poses[model_name]["position"]["z"] = pose.position.z
                    self.poses[model_name]["orientation"]["x"] = pose.orientation.x
                    self.poses[model_name]["orientation"]["y"] = pose.orientation.y
                    self.poses[model_name]["orientation"]["z"] = pose.orientation.z
                    self.poses[model_name]["orientation"]["w"] = pose.orientation.w
            
            if self.message_count == 1:
                print(f"First pose message: updated {models_updated}/{len(self.model_names)} models")
                if models_updated == 0:
                    print("No models were updated! Debugging model names in message:")
                    message_models = [pose.name for pose in msg.pose]
                    print(f"Models in message: {message_models[:10]}... (showing first 10)")
                    print(f"We're looking for: {self.model_names}")
        except Exception as e:
            print(f"Error in pose_cb: {str(e)}")

    def _start_debug_thread(self):
        """Start a thread to periodically print debug information"""
        def debug_loop():
            count = 0
            while True:
                time.sleep(5.0)  # Check every 5 seconds
                count += 1
                
                # Add connection status to debug info
                time_since_last = time.time() - self.last_message_time if self.last_message_time > 0 else -1
                print(f"\n--- GazeboPosesTracker Debug [{count}] ---")
                print(f"Gazebo connection verified: {self.connection_verified}")
                print(f"Total messages received: {self.message_count}")
                print(f"Time since last message: {time_since_last:.1f}s")
                print(f"Are poses valid: {self.are_poses_valid()}")
                
                if hasattr(self, 'use_direct_access') and self.use_direct_access:
                    print("Using direct model access (fallback mode)")
                
                # Additional diagnostics if no messages are received
                if self.message_count == 0:
                    print("No messages received yet. Potential issues:")
                    print(f"1. Incorrect world name (current: {self.world_name})")
                    print("2. Gazebo not publishing to expected topics")
                    print("3. Network or permission issues")
                    print("4. Model names mismatch")
                    
                    # If direct access is not set up yet and we've been waiting a while
                    if count > 2 and not hasattr(self, 'use_direct_access'):
                        print("No messages after multiple attempts - trying direct model access")
                        self._setup_direct_model_access()
                
                # Sample a random model to check its pose
                if self.model_names:
                    sample_model = self.model_names[0]
                    if sample_model in self.poses:
                        pos = self.poses[sample_model]["position"]
                        print(f"Sample model '{sample_model}' position: x={pos['x']}, y={pos['y']}, z={pos['z']}")
                
                # Try to resubscribe if no messages received
                if count % 2 == 0 and self.message_count == 0:
                    print("No messages received, attempting to resubscribe...")
                    self._subscribe_to_topics()
                
        # Start debug thread
        debug_thread = threading.Thread(target=debug_loop)
        debug_thread.daemon = True
        debug_thread.start()

    def print_pose_deltas(self):
        print("\n--- Current Poses ---")
        for model_name, pose in self.poses.items():
            print(f"Model: {model_name}")
            print(f"Position: {pose['position']}")
            print(f"Orientation: {pose['orientation']}")

# Update check_ros2_services to also show Gazebo connection status
def check_ros2_services(verbose=True, check_gazebo=True, gazebo_timeout=10.0):
    """
    Check available ROS2 services and return information about them.
    Useful for debugging service discovery issues.
    
    Args:
        verbose (bool): If True, print service information
        check_gazebo (bool): If True, also check Gazebo connection
        gazebo_timeout (float): Timeout for Gazebo commands
        
    Returns:
        list: List of available services
    """
    try:
        if verbose:
            print("\n--- Checking ROS2 Services ---")
        
        # Use subprocess to run ros2 service list
        result = subprocess.run(['ros2', 'service', 'list', '-t'], 
                                capture_output=True, text=True, check=True)
        
        services = []
        for line in result.stdout.strip().split('\n'):
            if line:
                parts = line.split(' [')
                if len(parts) >= 2:
                    service_name = parts[0].strip()
                    service_type = '[' + parts[1].strip()
                    services.append((service_name, service_type))
                    if verbose:
                        print(f"Service: {service_name} - Type: {service_type}")
                else:
                    if verbose:
                        print(f"Malformed service entry: {line}")
        
        if verbose:
            print(f"Total services found: {len(services)}")
            
            # Check specifically for update_health service
            update_health_services = [s for s in services if 'update_health' in s[0]]
            if update_health_services:
                print(f"Found update_health services: {update_health_services}")
            else:
                print("No update_health service found!")
        
        # Check Gazebo connection if requested
        if check_gazebo:
            if verbose:
                print("\n--- Checking Gazebo Status ---")
            
            # Check if gzserver is running
            try:
                ps_result = subprocess.run(['ps', '-ef'], capture_output=True, text=True)
                if 'gzserver' in ps_result.stdout or 'gz sim' in ps_result.stdout:
                    if verbose:
                        print("✓ Gazebo server is running")
                else:
                    if verbose:
                        print("✗ Gazebo server is NOT running")
            except Exception as e:
                if verbose:
                    print(f"Error checking Gazebo process: {e}")
            
            # Check Gazebo topics
            try:
                if verbose:
                    print(f"Running 'gz topic -l' with {gazebo_timeout}s timeout...")
                gz_result = subprocess.run(['gz', 'topic', '-l'], 
                                         capture_output=True, text=True, 
                                         timeout=gazebo_timeout)
                if gz_result.returncode == 0:
                    topics = gz_result.stdout.strip().split('\n')
                    if verbose:
                        print(f"Gazebo topics available: {len(topics)}")
                        
                        # Show some sample topics
                        if topics:
                            print("Sample topics:")
                            for topic in topics[:5]:  # Show first 5 topics
                                print(f"  {topic}")
                            if len(topics) > 5:
                                print(f"  ... and {len(topics)-5} more")
                else:
                    if verbose:
                        print(f"Error listing Gazebo topics: {gz_result.stderr}")
            except subprocess.TimeoutExpired:
                if verbose:
                    print(f"WARNING: 'gz topic -l' timed out after {gazebo_timeout} seconds")
                    print("This usually means Gazebo is overloaded or running on a slow machine")
                    print("The system will use direct model access as a fallback mechanism")
            except Exception as e:
                if verbose:
                    print(f"Error accessing Gazebo topics: {e}")
                    
        return services
    except Exception as e:
        if verbose:
            print(f"Error checking ROS2 services: {e}")
        return []

def main():
    robot_names = ["/px4_1", "/px4_2"]
    gazebo_poses = GazeboPosesTracker(robot_names, world_name="swarmz_world_2")
    try:
        def print_pose():
            for robot in robot_names:
                pose = gazebo_poses.get_pose(robot)
                print(f"\nCurrent Pose for {robot}:", pose)
            threading.Timer(1.0, print_pose).start()

        print_pose()
        
        # Check available ROS2 services after a brief delay
        time.sleep(5)
        check_ros2_services()
        
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()
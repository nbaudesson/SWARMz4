"""
Multi-drone Mission Controller for PX4

This node reads a mission file and automatically launches the appropriate controller
(NED or FRD) for each drone based on their configuration, then executes their missions
concurrently.

Features:
- Automatic controller selection and launching
- Concurrent mission execution
- Graceful shutdown and cleanup
- Mission progress tracking
- Hover time management
- Process monitoring

Mission File Format (YAML):
    "1":  # Drone ID
        spawn:  # Optional - If present, uses NED controller with absolute coordinates
            x: 0.0  # North position
            y: 0.0  # East position
        waypoints:  # Required - List of waypoints
            - x: 5.0    # Meters (absolute if NED, relative if FRD)
              y: 0.0    # Meters (absolute if NED, relative if FRD)
              z: -2.0   # Meters (negative is up)
              yaw: 0.0  # Degrees (absolute if NED, relative if FRD)
              hover: 5.0  # Seconds to hover at waypoint

Controller Selection:
    - If spawn coordinates are provided: Uses NED controller (absolute positioning)
    - If no spawn coordinates: Uses FRD controller (relative positioning)

Usage:
    1. Create a mission file in the package's missions directory
    2. Run the controller:
       ```bash
       ros2 run offboard_control_py offboard_multicontrol --ros-args -p mission_file:=test_mission
       ```

The node will:
    1. Load the mission file
    2. Launch appropriate controllers for each drone
    3. Execute all missions concurrently
    4. Clean up processes on shutdown

Notes:
    - Mission files must be in package's missions directory
    - .yaml extension is optional in command
    - Each drone executes independently
    - Controllers are auto-terminated on shutdown
"""

# Required ROS2 and system imports
import rclpy
from rclpy.node import Node
import subprocess
import signal
import os
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_controllers_interfaces.action import GotoPosition
import yaml
import asyncio
import time
from typing import Dict
import threading
from ament_index_python.packages import get_package_share_directory
import os.path
import atexit
import psutil

class DroneController:
    """Handles the execution of waypoint missions for a single drone."""
    
    def __init__(self, node: Node, drone_id: str, config: dict):
        # Store node reference and drone identification
        self.node = node
        self.drone_id = drone_id
        self.config = config
        
        # Mission progress tracking
        self.current_waypoint = 0
        self.total_waypoints = len(config['waypoints'])
        
        # Setup action client for this drone
        self.action_client = ActionClient(
            node,
            GotoPosition,
            f'/px4_{drone_id}/goto_position',
            callback_group=ReentrantCallbackGroup()
        )
        
        # Mission state tracking
        self.mission_complete = False
        self.current_goal_handle = None
        self.mission_start_time = None

    async def execute_mission(self):
        # Wait for action server to be available
        if not await self.action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(f'Action server not available for drone {self.drone_id}')
            return False
        
        # Start mission timing
        self.mission_start_time = time.time()
        
        # Process each waypoint sequentially
        while self.current_waypoint < self.total_waypoints and not self.mission_complete:
            waypoint = self.config['waypoints'][self.current_waypoint]
            
            # Log waypoint execution start
            self.node.get_logger().info(
                f'Drone {self.drone_id}: Waypoint {self.current_waypoint + 1}/{self.total_waypoints} '
                f'[{waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f}]'
            )
            
            # Create and send goal to controller
            goal_msg = GotoPosition.Goal()
            goal_msg.target.position.x = float(waypoint['x'])
            goal_msg.target.position.y = float(waypoint['y'])
            goal_msg.target.position.z = float(waypoint['z'])
            goal_msg.target.yaw = float(waypoint['yaw'])
            
            # Send goal and wait for acceptance
            goal_future = await self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            
            # Handle goal rejection
            if not goal_future.accepted:
                self.node.get_logger().error(f'Goal rejected for drone {self.drone_id}')
                return False
            
            # Store current goal and wait for completion    
            self.current_goal_handle = goal_future
            result = (await goal_future.get_result_async()).result
            
            # If waypoint reached, hover and move to next
            if result.success:
                await asyncio.sleep(waypoint['hover'])
                self.current_waypoint += 1
            else:
                return False
        
        # Mission completed successfully
        self.mission_complete = True
        return True

    def feedback_callback(self, feedback_msg):
        # Log distance to current waypoint
        if self.current_waypoint < self.total_waypoints:
            self.node.get_logger().debug(
                f'Drone {self.drone_id}: Distance to waypoint: '
                f'{feedback_msg.feedback.distance_to_target:.1f}m'
            )

class MultiDroneMissionController(Node):
    """Main node that manages multiple drone missions."""
    
    def __init__(self):
        super().__init__('offboard_multicontrol')
        
        # Register cleanup handlers
        atexit.register(self.cleanup_all)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Create asyncio event loop
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
        
        # Store processes for cleanup
        self.controller_processes = {}
        
        # Get mission file name from parameter
        self.declare_parameter('mission_file', '')
        mission_name = self.get_parameter('mission_file').value
        
        if not mission_name:
            self.get_logger().error('No mission file specified')
            return
            
        # Find mission file in package's missions directory
        try:
            # Get package share directory
            package_dir = get_package_share_directory('offboard_control_py')
            missions_dir = os.path.join(package_dir, 'missions')
            
            # Add .yaml extension if not present
            if not mission_name.endswith('.yaml'):
                mission_name += '.yaml'
                
            # Construct full path
            mission_path = os.path.join(missions_dir, mission_name)
            
            if not os.path.exists(mission_path):
                self.get_logger().error(f'Mission file not found: {mission_name}')
                return
                
            self.get_logger().info(f'Loading mission from: {mission_name}')
            
            # Load mission file
            with open(mission_path, 'r') as f:
                self.mission_config = yaml.safe_load(f)
                
        except Exception as e:
            self.get_logger().error(f'Failed to load mission file: {str(e)}')
            return
            
        # Launch controller nodes first
        self.launch_controllers()
        
        # Create controller instances for each drone
        self.controllers: Dict[str, DroneController] = {}
        for drone_id, config in self.mission_config.items():
            self.controllers[drone_id] = DroneController(self, drone_id, config)
        
        # Start mission execution using the loop
        self.loop.create_task(self.execute_all_missions())

    def launch_controllers(self):
        """Launch appropriate controller node for each drone."""
        for drone_id, config in self.mission_config.items():
            # Determine controller type based on spawn coordinates presence
            has_spawn = 'spawn' in config
            namespace = f'__ns:=/px4_{drone_id}'
            
            # Prepare launch command
            cmd = ['ros2', 'run', 'offboard_control_py']
            if has_spawn:
                # Has spawn coordinates -> Use NED controller
                spawn = config['spawn']
                cmd.extend([
                    'offboard_control_ned',
                    '--ros-args', '-r', namespace,
                    '-p', f'spawn_x:={spawn.get("x", 0.0)}',
                    '-p', f'spawn_y:={spawn.get("y", 0.0)}'
                ])
            else:
                # No spawn coordinates -> Use FRD controller
                cmd.extend(['offboard_control_frd', '--ros-args', '-r', namespace])
                
            # Launch controller process
            try:
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self.controller_processes[drone_id] = process
                self.get_logger().info(
                    f'Launched {"NED" if has_spawn else "FRD"} controller '
                    f'for drone {drone_id}'
                )
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f'Failed to launch controller for drone {drone_id}: {str(e)}')

    def cleanup_all(self):
        """Thorough cleanup of all processes."""
        self.get_logger().info('Cleaning up all controller processes...')
        
        try:
            # First try graceful termination of our known processes
            for drone_id, process in self.controller_processes.items():
                try:
                    if process.poll() is None:  # Process is still running
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        self.get_logger().info(f'Sent SIGTERM to controller {drone_id}')
                except Exception as e:
                    self.get_logger().error(f'Error terminating controller {drone_id}: {str(e)}')
            
            # Give processes time to terminate gracefully
            time.sleep(1)
            
            # Force kill any remaining processes
            for drone_id, process in self.controller_processes.items():
                try:
                    if process.poll() is None:  # Process is still running
                        pgid = os.getpgid(process.pid)
                        os.killpg(pgid, signal.SIGKILL)
                        self.get_logger().warn(f'Force killed controller {drone_id}')
                except Exception:
                    pass
            
            # Find and kill any orphaned controller processes
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.cmdline()
                    if len(cmdline) >= 2 and 'offboard_control' in cmdline[1]:
                        proc.kill()
                        self.get_logger().warn(f'Killed orphaned controller: {cmdline[1]}')
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
                
        except Exception as e:
            self.get_logger().error(f'Error in cleanup: {str(e)}')
            
        self.get_logger().info('Cleanup complete')

    def signal_handler(self, signum, frame):
        """Handle termination signals."""
        self.get_logger().info(f'Received signal {signum}')
        self.cleanup_all()
        sys.exit(0)

    def cleanup(self):
        """Terminate all controller processes on shutdown."""
        self.cleanup_all()

    async def execute_all_missions(self):
        """Execute all drone missions concurrently."""
        # Create tasks for each drone's mission
        tasks = []
        for controller in self.controllers.values():
            tasks.append(asyncio.create_task(controller.execute_mission()))
            
        # Wait for all missions to complete
        await asyncio.gather(*tasks, return_exceptions=True)

def main(args=None):
    """Main function handling node lifecycle."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = MultiDroneMissionController()
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        
        try:
            node.loop.run_forever()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down...')
        except Exception as e:
            node.get_logger().error(f'Error in main loop: {str(e)}')
        finally:
            if node:
                node.cleanup_all()
                node.loop.stop()
                node.loop.close()
            
    except Exception as e:
        print(f'Error in main: {str(e)}')
        if node:
            node.cleanup_all()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Robot Position Monitor
=====================

A utility tool that tracks robot positions in a Gazebo simulation.
It gets position data from two sources:
1. Directly from Gazebo via the GazeboPosesTracker
2. From the game_master node via ROS2 topic

This dual approach ensures position data is always available, even if one source fails.

Usage:
    ros2 run game_master position_monitor [--world_name WORLD_NAME] [--debug]

Press Ctrl+C to exit.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import math
import os
import sys
import argparse
from utils.tools import get_stable_namespaces
from utils.gazebo_subscriber import GazeboPosesTracker

class PositionMonitor(Node):
    def __init__(self, world_name="swarmz_world_2", enable_debug=False):
        super().__init__('position_monitor')
        
        # Save parameters
        self.world_name = world_name
        self.enable_debug = enable_debug
        
        # Initialize logging system
        self.log_buffer = []
        self.max_log_lines = 15  # Keep last 15 log messages
        
        # Print header only once
        self._print_header()
        self.log(f"World: {self.world_name}")
        self.log(f"Debug: {'Enabled' if self.enable_debug else 'Disabled'}")
        
        # Initialize counters and state variables
        self.last_direct_update_time = 0
        self.last_topic_update_time = 0
        self.direct_positions = {}
        self.topic_positions = {}
        self.last_direct_positions = {}
        self.last_topic_positions = {}
        self.initialized = False
        self.namespaces = []
        
        # Configure flexible QoS for better compatibility
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize ROS2 communication
        self.subscription = self.create_subscription(
            String,
            '/game_master/debug_positions',
            self.topic_callback,
            qos_profile
        )
        
        # Create a heartbeat publisher for diagnostics
        self.heartbeat_publisher = self.create_publisher(
            String,
            '/position_monitor/heartbeat',
            10
        )
        
        # Setup display and update timers
        self.display_timer = self.create_timer(1.0, self.display_positions)
        self.heartbeat_timer = self.create_timer(2.0, self.send_heartbeat)
        self.diagnostics_timer = self.create_timer(5.0, self.check_diagnostics)
        
        # Initialize Gazebo connection
        self.log("Initializing Gazebo connection...")
        self.log("Detecting robot namespaces...")
        self.direct_init_success = False
        
        # First try to get namespaces from ROS2
        try:
            self.namespaces = get_stable_namespaces(self, max_attempts=3, wait_time=0.5)
            if self.namespaces:
                self.log(f"Found {len(self.namespaces)} namespaces from ROS2")
                for ns in self.namespaces[:5]:
                    self.log(f"  - {ns}")
                if len(self.namespaces) > 5:
                    self.log(f"  - ... and {len(self.namespaces)-5} more")
            else:
                self.log("No namespaces found from ROS2", level="WARN")
        except Exception as e:
            self.log(f"Error getting namespaces from ROS2: {e}", level="ERROR")
            self.namespaces = []
        
        # If we couldn't get namespaces, provide some default ones
        if not self.namespaces:
            # Default robot list when no robots are detected
            self.namespaces = [f"/px4_{i}" for i in range(10)]
            self.log(f"Using default namespaces: {self.namespaces}")
        
        # Initialize direct connection to Gazebo
        try:
            self.gazebo_tracker = GazeboPosesTracker(
                self.namespaces,
                world_name=self.world_name,
                enable_debug=self.enable_debug,
                debug_interval=5.0
            )
            self.log("Created GazeboPosesTracker for direct Gazebo connection")
            
            # Add a timer for updating positions from Gazebo
            self.gazebo_update_timer = self.create_timer(0.5, self.update_from_gazebo)
            self.direct_init_success = True
        except Exception as e:
            self.log(f"Error initializing direct Gazebo connection: {e}", level="ERROR")
            self.log("Will try to get positions only from game_master topic", level="WARN")
    
    def log(self, message, level="INFO"):
        """Add a message to the log buffer with timestamp"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[{timestamp}] [{level}] {message}"
        
        # Add to log buffer
        self.log_buffer.append(log_entry)
        
        # Trim buffer if it gets too large
        if len(self.log_buffer) > self.max_log_lines:
            self.log_buffer = self.log_buffer[-self.max_log_lines:]
        
        # Also print to console for debugging
        print(log_entry)

    def _print_header(self):
        """Print the header for the position monitor"""
        print("\n" + "=" * 80)
        print(" Robot Position Monitor ".center(80, "="))
        print("=" * 80)

    def _print_log_section(self):
        """Print the log section that stays at the top"""
        print("\n" + "-" * 80)
        print(" Log Messages ".center(80, "-"))
        print("-" * 80)
        
        for entry in self.log_buffer:
            print(entry)
        
        print("-" * 80)

    def update_from_gazebo(self):
        """Get latest positions directly from Gazebo"""
        if not hasattr(self, 'gazebo_tracker'):
            return
            
        try:
            # Check if poses are valid
            if not self.gazebo_tracker.are_poses_valid():
                if time.time() - self.last_direct_update_time > 5.0:
                    self.log("No valid poses from direct Gazebo connection", level="WARN")
                return
                
            # Store the last positions before updating
            if self.direct_positions:
                self.last_direct_positions = self.direct_positions.copy()
            
            # Get new positions
            positions = {}
            for ns in self.namespaces:
                pose = self.gazebo_tracker.get_pose(ns)
                if pose['position']['x'] is not None:
                    positions[ns] = {
                        "position": {
                            "x": pose['position']['x'],
                            "y": pose['position']['y'],
                            "z": pose['position']['z']
                        },
                        "orientation": {
                            "x": pose['orientation']['x'],
                            "y": pose['orientation']['y'],
                            "z": pose['orientation']['z'],
                            "w": pose['orientation']['w']
                        }
                    }
            
            # Only update if we got some positions
            if positions:
                self.direct_positions = positions
                self.last_direct_update_time = time.time()
                
                # Log first successful update
                if not self.initialized:
                    self.initialized = True
                    self.log(f"First positions from Gazebo received for {len(positions)} robots", level="INFO")
        except Exception as e:
            self.log(f"Error updating from Gazebo: {e}", level="ERROR")
    
    def topic_callback(self, msg):
        """Process position data from game_master topic"""
        try:
            data = json.loads(msg.data)
            if data:
                # Store the last positions before updating
                if self.topic_positions:
                    self.last_topic_positions = self.topic_positions.copy()
                
                # Update with new data
                self.topic_positions = data
                self.last_topic_update_time = time.time()
                
                # Print info on first successful message
                if not hasattr(self, '_first_topic_msg'):
                    self._first_topic_msg = True
                    self.log(f"First positions from topic received for {len(data)} robots", level="INFO")
        except Exception as e:
            self.log(f"Error processing topic data: {e}", level="ERROR")
    
    def send_heartbeat(self):
        """Send a heartbeat message for diagnostics"""
        try:
            msg = String()
            msg.data = f"Heartbeat: {time.time()}"
            self.heartbeat_publisher.publish(msg)
        except Exception as e:
            self.log(f"Error sending heartbeat: {e}", level="ERROR")
    
    def check_diagnostics(self):
        """Run diagnostics to check system status"""
        # Skip first couple of runs to give time for connections
        if not hasattr(self, '_diagnostics_count'):
            self._diagnostics_count = 0
        self._diagnostics_count += 1
        
        if self._diagnostics_count <= 2:
            return
            
        # Only run diagnostics if we're not receiving data
        direct_working = time.time() - self.last_direct_update_time < 5.0
        topic_working = time.time() - self.last_topic_update_time < 5.0
        
        if not direct_working and not topic_working:
            self.log("No position data received from either source!", level="ERROR")
            
            if hasattr(self, 'gazebo_tracker'):
                connection_status = getattr(self.gazebo_tracker, 'connection_verified', False)
                message_count = getattr(self.gazebo_tracker, 'message_count', 0)
                self.log(f"Gazebo connection verified: {connection_status}", level="INFO")
                self.log(f"Messages received from Gazebo: {message_count}", level="INFO")
            
            # Check topic info
            try:
                import subprocess
                topic_result = subprocess.run(
                    ['ros2', 'topic', 'info', '/game_master/debug_positions'],
                    capture_output=True, text=True, timeout=2.0
                )
                
                if topic_result.returncode == 0:
                    topic_info = topic_result.stdout.strip()
                    self.log(f"Topic info: {topic_info}", level="INFO")
                    
                    # Extract publisher count
                    pub_count = 0
                    if "Publisher count: " in topic_info:
                        pub_count_line = [line for line in topic_info.split('\n') if "Publisher count: " in line]
                        if pub_count_line:
                            pub_count = int(pub_count_line[0].split("Publisher count: ")[1])
                    
                    if pub_count == 0:
                        self.log("No publishers for /game_master/debug_positions", level="WARN")
                else:
                    self.log(f"Error getting topic info: {topic_result.stderr.strip()}", level="ERROR")
                    
                # Check if game_master_node is running
                node_result = subprocess.run(
                    ['ros2', 'node', 'list'],
                    capture_output=True, text=True, timeout=2.0
                )
                if 'game_master_node' in node_result.stdout:
                    self.log("Game master node is running", level="INFO")
                else:
                    self.log("Game master node is NOT running!", level="ERROR")
                    self.log("Start it with: ros2 launch game_master game_master.launch.py debug_gazebo:=true", level="INFO")
            except Exception as e:
                self.log(f"Error checking ROS2 status: {e}", level="ERROR")
            
            self.log("Possible solutions:", level="INFO")
            self.log("1. Make sure Gazebo is running", level="INFO")
            self.log("2. Start game_master with: ros2 launch game_master game_master.launch.py debug_gazebo:=true", level="INFO")
            self.log("3. Check if the world name matches the running simulation", level="INFO")
            self.log(f"   Current world name: {self.world_name}", level="INFO")
            self.log("4. Restart this node with correct world name:", level="INFO")
            self.log(f"   ros2 run game_master position_monitor --world_name YOUR_WORLD_NAME", level="INFO")
    
    def display_positions(self):
        """Display the position data in the terminal"""
        # Choose which data source to use - prioritize direct connection for freshness
        positions = self.direct_positions if self.direct_positions else self.topic_positions
        last_positions = self.last_direct_positions if self.direct_positions else self.last_topic_positions
        
        # If we have no positions, show waiting message
        if not positions:
            # Only show this message occasionally to avoid flooding
            if not hasattr(self, '_wait_count'):
                self._wait_count = 0
            self._wait_count += 1
            
            if self._wait_count % 5 == 0:
                self.log("Waiting for position data...", level="WARN")
            return
        
        # Clear just the display portion, not the logs at the top
        print("\033[2J\033[H", end="")  # Clear screen and move cursor to top
        
        # Print log section at the top
        self._print_log_section()
        
        # Print header
        print("\n" + "=" * 80)
        print(" Robot Position Monitor ".center(80, "="))
        print("=" * 80)
        
        # Print data source info
        direct_age = time.time() - self.last_direct_update_time if self.last_direct_update_time > 0 else float('inf')
        topic_age = time.time() - self.last_topic_update_time if self.last_topic_update_time > 0 else float('inf')
        
        using_direct = direct_age < topic_age
        print(f"Data source: {'Direct Gazebo' if using_direct else 'Game Master Topic'}")
        print(f"Direct data age: {direct_age:.1f}s ({len(self.direct_positions)} robots)")
        print(f"Topic data age: {topic_age:.1f}s ({len(self.topic_positions)} robots)")
        print(f"Monitoring {len(positions)} robots\n")
        
        # Sort robots by type (drones first, then ships)
        sorted_robots = sorted(positions.keys(), 
                              key=lambda x: (0 if 'px4' in x else 1, x))
        
        # Print drone positions
        print("-" * 80)
        print(" Drones ".center(80, "-"))
        print("-" * 80)
        print(f"{'Robot':^12} | {'Position (x, y, z)':^30} | {'Velocity':^10} | {'Status':^15}")
        print("-" * 80)
        
        for robot in sorted_robots:
            if 'px4' not in robot:
                continue
                
            pos = positions[robot]['position']
            
            # Calculate velocity if we have previous positions
            velocity = "N/A"
            status = "Stationary"
            if robot in last_positions:
                last_pos = last_positions[robot]['position']
                dx = pos['x'] - last_pos['x']
                dy = pos['y'] - last_pos['y']
                dz = pos['z'] - last_pos['z']
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > 0.01:  # If moved more than 1cm
                    velocity = f"{dist:.2f} m/s"
                    status = "Moving"
            
            print(f"{robot:^12} | ({pos['x']:6.2f}, {pos['y']:6.2f}, {pos['z']:6.2f}) | {velocity:^10} | {status:^15}")
        
        # Print ship positions
        print("\n" + "-" * 80)
        print(" Ships ".center(80, "-"))
        print("-" * 80)
        print(f"{'Robot':^12} | {'Position (x, y, z)':^30} | {'Status':^15}")
        print("-" * 80)
        
        for robot in sorted_robots:
            if 'px4' in robot:
                continue
                
            pos = positions[robot]['position']
            
            # Determine status
            status = "Stationary"
            if robot in last_positions:
                last_pos = last_positions[robot]['position']
                dx = pos['x'] - last_pos['x']
                dy = pos['y'] - last_pos['y']
                dz = pos['z'] - last_pos['z']
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist > 0.01:  # If moved more than 1cm
                    status = "Moving"
            
            print(f"{robot:^12} | ({pos['x']:6.2f}, {pos['y']:6.2f}, {pos['z']:6.2f}) | {status:^15}")
        
        # Show data comparison if both sources are available
        if self.direct_positions and self.topic_positions:
            print("\n" + "-" * 80)
            print(" Data Source Comparison ".center(80, "-"))
            print("-" * 80)
            
            # Find common robots in both data sources
            common_robots = set(self.direct_positions.keys()) & set(self.topic_positions.keys())
            if common_robots:
                sample_robot = next(iter(common_robots))
                direct_pos = self.direct_positions[sample_robot]['position']
                topic_pos = self.topic_positions[sample_robot]['position']
                
                # Calculate difference
                diff_x = direct_pos['x'] - topic_pos['x']
                diff_y = direct_pos['y'] - topic_pos['y']
                diff_z = direct_pos['z'] - topic_pos['z']
                total_diff = math.sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z)
                
                print(f"Sample robot: {sample_robot}")
                print(f"Direct: ({direct_pos['x']:.2f}, {direct_pos['y']:.2f}, {direct_pos['z']:.2f})")
                print(f"Topic:  ({topic_pos['x']:.2f}, {topic_pos['y']:.2f}, {topic_pos['z']:.2f})")
                print(f"Difference: {total_diff:.4f} meters")
            else:
                print("No common robots found between data sources")
        
        print("\nPress Ctrl+C to exit.")

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot Position Monitor')
    parser.add_argument('--world_name', type=str, default='swarmz_world_2',
                        help='Name of the Gazebo world (default: swarmz_world_2)')
    parser.add_argument('--debug', action='store_true', 
                        help='Enable debug output')
    return parser.parse_args()

def main(args=None):
    """Main entry point"""
    # Parse command-line args and ROS args
    if args is None:
        args = sys.argv[1:]
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Parse our custom arguments
    parsed_args = parse_args()
    
    try:
        # Create and run the position monitor
        monitor = PositionMonitor(
            world_name=parsed_args.world_name,
            enable_debug=parsed_args.debug
        )
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nExiting Position Monitor.")
    except Exception as e:
        print(f"Error in position monitor: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

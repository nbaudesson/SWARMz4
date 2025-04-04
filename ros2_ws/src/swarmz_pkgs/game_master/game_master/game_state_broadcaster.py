#!/usr/bin/env python3
"""
Game State Broadcaster
=====================

Collects and broadcasts game state data for external visualization in Unity.
Acts as a WebSocket server that sends JSON-formatted game state updates.

Data broadcast includes:
- Robot positions from Gazebo
- Robot health points
- Remaining game time
- Missile firing events
- Ammunition counts
- Team scores and assignments

Usage:
    ros2 run game_master game_state_broadcaster

Configuration:
    websocket_port: Port number for WebSocket server (default: 9090)
    broadcast_rate: How often to broadcast updates (default: 0.1 seconds = 10Hz)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import json
import time
import threading
import logging
from functools import partial

# Import websocket_server and handle missing dependency
try:
    import websocket_server
except ImportError:
    print("ERROR: websocket_server module not found. Please install it with:")
    print("pip install websocket-server")
    print("Then restart this node.")
    raise

class GameStateBroadcaster(Node):
    def __init__(self):
        super().__init__('game_state_broadcaster')
        
        # Configure logging
        logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)
        
        # Declare parameters
        self.declare_parameter('websocket_port', 9090)
        self.declare_parameter('broadcast_rate', 0.1)  # 10Hz default
        
        # Get parameter values
        self.websocket_port = self.get_parameter('websocket_port').get_parameter_value().integer_value
        self.broadcast_rate = self.get_parameter('broadcast_rate').get_parameter_value().double_value
        
        # Initialize data structures
        self.robot_positions = {}
        self.robot_health = {}
        self.robot_ammo = {}
        self.team_assignments = {"team_1": [], "team_2": []}
        self.team_scores = {"team_1": 0, "team_2": 0}
        self.remaining_time = 0
        self.firing_events = []  # List of recent firing events with timestamp expiration
        self.kamikaze_events = []  # List of recent kamikaze events with timestamp expiration
        self.clients_connected = 0
        
        # Track when data fields were last updated
        self.last_update = {
            "positions": 0,
            "health": 0,
            "time": 0,
            "ammo": 0,
            "teams": 0
        }
        
        # Subscribe to topics
        self.create_subscription(
            String,
            '/game_master/debug_positions',
            self.position_callback,
            10
        )
        
        self.create_subscription(
            Int32,
            '/game_master/time',
            self.time_callback,
            10
        )
        
        # Add subscription for team data
        self.create_subscription(
            String,
            '/game_master/team_data',
            self.team_data_callback,
            10
        )
        
        # Add subscription for missile events
        self.create_subscription(
            String,
            '/game_master/missile_events',
            self.missile_events_callback,
            10
        )
        
        # Set up the WebSocket server only if websocket_server exists
        self.setup_websocket_server()
        
        # Create a timer to broadcast game state
        self.broadcast_timer = self.create_timer(
            self.broadcast_rate, 
            self.broadcast_game_state
        )
        
        # Create timer to discover and subscribe to robot-specific topics
        self.discovery_timer = self.create_timer(5.0, self.discover_robot_topics)
        
        # Create timer to clean up old events
        self.cleanup_timer = self.create_timer(5.0, self.cleanup_events)
        
        # Dynamic subscriptions (will be populated by discover_robot_topics)
        self.health_subscriptions = {}
        self.ammo_subscriptions = {}
        
        self.get_logger().info("Game State Broadcaster initialized and ready")
    
    def setup_websocket_server(self):
        """Set up the WebSocket server"""
        self.get_logger().info(f"Starting WebSocket server on port {self.websocket_port}")
        try:
            self.server = websocket_server.WebsocketServer(
                host='0.0.0.0', 
                port=self.websocket_port,
                loglevel=logging.INFO
            )
            
            # Set WebSocket callbacks
            self.server.set_fn_new_client(self.new_client)
            self.server.set_fn_client_left(self.client_left)
            self.server.set_fn_message_received(self.message_received)
            
            # Start WebSocket server in a separate thread
            self.server_thread = threading.Thread(target=self.server.run_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
        except Exception as e:
            self.get_logger().error(f"Failed to start WebSocket server: {e}")
            self.get_logger().error("Unity visualization will not be available")
    
    def new_client(self, client, server):
        """Callback for new WebSocket client connection"""
        self.clients_connected += 1
        self.get_logger().info(f"New client connected (total: {self.clients_connected})")
        
        # Send initial game state to the new client
        self.send_full_state_to_client(client)
    
    def client_left(self, client, server):
        """Callback for client disconnection"""
        self.clients_connected -= 1
        self.get_logger().info(f"Client disconnected (remaining: {self.clients_connected})")
    
    def message_received(self, client, server, message):
        """Callback for message from client"""
        try:
            data = json.loads(message)
            if 'request' in data:
                if data['request'] == 'full_state':
                    self.send_full_state_to_client(client)
                elif data['request'] == 'robot_list':
                    robot_list = list(self.robot_positions.keys())
                    server.send_message(client, json.dumps({
                        'type': 'robot_list',
                        'robots': robot_list
                    }))
        except json.JSONDecodeError:
            self.get_logger().warn(f"Received invalid JSON: {message}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")
    
    def send_full_state_to_client(self, client):
        """Send the full game state to a specific client"""
        try:
            full_state = self.build_game_state(include_all=True)
            self.server.send_message(client, json.dumps(full_state))
        except Exception as e:
            self.get_logger().error(f"Error sending state to client: {e}")
    
    def position_callback(self, msg):
        """Process position data from game master"""
        try:
            data = json.loads(msg.data)
            self.robot_positions = data
            self.last_update["positions"] = time.time()
            
            # Extract robot list from positions if this is the first time
            if not self.team_assignments["team_1"] and not self.team_assignments["team_2"]:
                self.get_logger().info(f"Discovered {len(data)} robots")
                self.discover_robot_topics()
        except Exception as e:
            self.get_logger().error(f"Error processing position data: {str(e)}")
    
    def time_callback(self, msg):
        """Process remaining time data"""
        self.remaining_time = msg.data
        self.last_update["time"] = time.time()
    
    def team_data_callback(self, msg):
        """Process team assignment and score data"""
        try:
            data = json.loads(msg.data)
            if 'team_1' in data and 'team_2' in data:
                self.team_assignments["team_1"] = data['team_1'].get('members', [])
                self.team_assignments["team_2"] = data['team_2'].get('members', [])
                self.team_scores["team_1"] = data['team_1'].get('score', 0)
                self.team_scores["team_2"] = data['team_2'].get('score', 0)
                self.last_update["teams"] = time.time()
                self.get_logger().debug(f"Updated team data: Team 1 ({len(self.team_assignments['team_1'])} members), Team 2 ({len(self.team_assignments['team_2'])} members)")
        except Exception as e:
            self.get_logger().error(f"Error processing team data: {str(e)}")
    
    def missile_events_callback(self, msg):
        """Process missile firing events"""
        try:
            event = json.loads(msg.data)
            if event.get('type') == 'missile':
                # Add timestamp for expiration
                event['timestamp'] = time.time()
                self.firing_events.append(event)
                self.get_logger().info(f"Missile fired: {event.get('robot')} -> {event.get('target', 'unknown')}")
            elif event.get('type') == 'kamikaze':
                # Add timestamp for expiration
                event['timestamp'] = time.time()
                self.kamikaze_events.append(event)
                self.get_logger().info(f"Kamikaze activated: {event.get('robot')}")
        except Exception as e:
            self.get_logger().error(f"Error processing missile event: {str(e)}")
    
    def health_callback(self, msg, robot_name):
        """Process health updates for a specific robot"""
        self.robot_health[robot_name] = msg.data
        self.last_update["health"] = time.time()
    
    def ammo_callback(self, msg, robot_name):
        """Process ammunition updates for a specific robot"""
        self.robot_ammo[robot_name] = msg.data
        self.last_update["ammo"] = time.time()
    
    def cleanup_events(self):
        """Remove old events that have expired"""
        current_time = time.time()
        # Keep events from the last 10 seconds
        self.firing_events = [e for e in self.firing_events 
                             if current_time - e.get('timestamp', 0) < 10.0]
        self.kamikaze_events = [e for e in self.kamikaze_events 
                               if current_time - e.get('timestamp', 0) < 10.0]
    
    def discover_robot_topics(self):
        """Discover and subscribe to robot-specific topics"""
        if not self.robot_positions:
            return
            
        # Get list of robots from position data
        robot_list = list(self.robot_positions.keys())
        
        # Subscribe to health topics for each robot
        for robot in robot_list:
            if robot not in self.health_subscriptions:
                try:
                    self.health_subscriptions[robot] = self.create_subscription(
                        Int32,
                        f'{robot}/health',
                        partial(self.health_callback, robot_name=robot),
                        10
                    )
                    self.get_logger().info(f"Subscribed to {robot}/health")
                except Exception as e:
                    self.get_logger().warn(f"Failed to subscribe to {robot}/health: {str(e)}")
            
            # Try to subscribe to ammunition topics (only for drones)
            if 'px4' in robot and robot not in self.ammo_subscriptions:
                try:
                    # Note: This might need adjustment based on the actual topic structure
                    self.ammo_subscriptions[robot] = self.create_subscription(
                        Int32,
                        f'{robot}/ammo',
                        partial(self.ammo_callback, robot_name=robot),
                        10
                    )
                    self.get_logger().info(f"Subscribed to {robot}/ammo")
                except Exception as e:
                    self.get_logger().debug(f"No ammo topic for {robot}: {str(e)}")
    
    def build_game_state(self, include_all=False):
        """Build the game state JSON object"""
        current_time = time.time()
        
        # Basic state always included
        state = {
            "timestamp": current_time,
            "remaining_time": self.remaining_time,
            "robots": {}
        }
        
        # Add position data if available
        if self.robot_positions:
            for robot, pose_data in self.robot_positions.items():
                if robot not in state["robots"]:
                    state["robots"][robot] = {}
                
                state["robots"][robot]["position"] = pose_data["position"]
                state["robots"][robot]["orientation"] = pose_data["orientation"]
                
                # Add type information
                state["robots"][robot]["type"] = "drone" if "px4" in robot else "ship"
        
        # Add health data
        for robot, health in self.robot_health.items():
            if robot not in state["robots"]:
                state["robots"][robot] = {}
            state["robots"][robot]["health"] = health
        
        # Add ammo data
        for robot, ammo in self.robot_ammo.items():
            if robot not in state["robots"]:
                state["robots"][robot] = {}
            state["robots"][robot]["ammo"] = ammo
        
        # Add team info if available
        if self.team_assignments["team_1"] or self.team_assignments["team_2"]:
            state["teams"] = {
                "team_1": {
                    "members": self.team_assignments["team_1"],
                    "score": self.team_scores["team_1"]
                },
                "team_2": {
                    "members": self.team_assignments["team_2"],
                    "score": self.team_scores["team_2"]
                }
            }
        
        # Add recent events if any
        if self.firing_events:
            # Just send the last 10 events to avoid too much data
            state["firing_events"] = self.firing_events[-10:]
        
        if self.kamikaze_events:
            # Just send the last 10 events
            state["kamikaze_events"] = self.kamikaze_events[-10:]
        
        # For complete state, include data freshness
        if include_all:
            state["data_freshness"] = {
                "positions": current_time - self.last_update["positions"] if self.last_update["positions"] > 0 else -1,
                "health": current_time - self.last_update["health"] if self.last_update["health"] > 0 else -1,
                "time": current_time - self.last_update["time"] if self.last_update["time"] > 0 else -1,
                "ammo": current_time - self.last_update["ammo"] if self.last_update["ammo"] > 0 else -1,
                "teams": current_time - self.last_update["teams"] if self.last_update["teams"] > 0 else -1
            }
        
        return state
    
    def broadcast_game_state(self):
        """Broadcast game state to all connected clients"""
        if not hasattr(self, 'server') or self.clients_connected == 0:
            return  # No server or no clients, no need to build and send state
            
        try:
            state = self.build_game_state()
            message = json.dumps(state)
            self.server.send_message_to_all(message)
        except Exception as e:
            self.get_logger().error(f"Error broadcasting game state: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        broadcaster = GameStateBroadcaster()
        rclpy.spin(broadcaster)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
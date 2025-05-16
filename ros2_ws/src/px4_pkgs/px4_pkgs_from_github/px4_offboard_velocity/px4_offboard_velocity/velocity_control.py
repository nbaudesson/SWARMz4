#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from pymavlink import mavutil

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool


def convert_velocity_to_ned(velocity_cmd, current_attitude, coordinate_system):
    """
    Convert velocity commands from various coordinate systems to NED (North-East-Down)
    
    Parameters:
    velocity_cmd - Vector3 containing velocity command in source coordinate system
    current_attitude - current yaw angle in radians
    coordinate_system - string identifier of source system ('FLU', 'FRD', 'NED')
    
    Returns:
    Vector3 containing velocity in NED coordinate system
    """
    result = Vector3()
    yaw = current_attitude  # Current yaw angle in radians
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    if coordinate_system == 'FRD':
        # FRD (Forward-Right-Down) to NED (North-East-Down)
        # 1. Rotate from body-frame to world-frame
        # Note: Right in FRD is opposite of Left in FLU
        
        # First convert from body to world
        body_forward = velocity_cmd.x
        body_right = velocity_cmd.y
        body_down = velocity_cmd.z
        
        # Convert to NED
        # North = Forward component projected onto North axis + Right component projected onto North axis
        result.x = body_forward * cos_yaw + body_right * sin_yaw
        # East = Forward component projected onto East axis - Right component projected onto East axis
        result.y = body_forward * sin_yaw - body_right * cos_yaw
        # Down = Down (already aligned)
        result.z = body_down
        
    elif coordinate_system == 'NED':
        # Already in NED coordinate system, no conversion needed
        result.x = velocity_cmd.x
        result.y = velocity_cmd.y
        result.z = velocity_cmd.z
        
    else:
        # Default to FLU conversion if coordinate system is 'FLU' or not recognized
        # This maintains backward compatibility and makes FLU the default fallback
        body_forward = velocity_cmd.x
        body_left = velocity_cmd.y
        body_up = velocity_cmd.z
        
        result.x = body_forward * cos_yaw - body_left * sin_yaw
        result.y = body_forward * sin_yaw + body_left * cos_yaw
        result.z = -body_up
    
    return result


class OffboardControl(Node):
    """
    A ROS2 node for controlling PX4 drone velocity in offboard mode.
    This node implements a state machine for arming, takeoff, and velocity control.
    """

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Parameters
        self.declare_parameter('coordinate_system', 'FLU')
        self._coordinate_system = self.get_parameter('coordinate_system').value

        #Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)


        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Initialize MAVLink connection
        try:
            self.mavlink_connection = mavutil.mavlink_connection('udpin:localhost:14540')
            self.get_logger().info("MAVLink connection established")
        except:
            self.get_logger().error("Failed to establish MAVLink connection")
            self.mavlink_connection = None

        # State machine variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX  # Current navigation state of the drone
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED    # Current arming state
        self.velocity = Vector3()                            # Desired velocity commands
        self.yaw = 0.0                                      # Desired yaw command
        self.trueYaw = 0.0                                  # Current yaw of the drone
        self.offboardMode = False                           # Flag for offboard mode status
        self.flightCheck = False                            # Flag for pre-flight checks
        self.myCnt = 0                                      # Counter for state machine timing
        self.arm_message = False                            # Flag for arm command
        self.failsafe = False                               # Flag for failsafe status
        self.current_state = "IDLE"                         # Current state in state machine
        self.last_state = self.current_state                # Previous state

    # Callback for arm message subscriber
    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    def arm_timer_callback(self):
        """
        State machine implementation for drone control.
        States:
        - IDLE: Waiting for flight checks and arm command
        - ARMING: Sending arm commands
        - TAKEOFF: Initiating takeoff procedure
        - LOITER: Waiting in hover state
        - OFFBOARD: Active velocity control mode
        """

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        """Switch to offboard mode for velocity control"""
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True   

    def arm(self):
        """Send arm command to the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    def take_off(self):
        """Command the vehicle to take off to specified altitude"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """
        Publish command to the vehicle
        param1, param2: Command-specific parameters
        param7: Used as altitude for takeoff command
        """
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_status_callback(self, msg):
        """
        Handle vehicle status updates and monitor:
        - Navigation state
        - Arming state
        - Failsafe status
        - Pre-flight checks
        """

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass


    def offboard_velocity_callback(self, msg):
        """
        Convert velocity commands from the configured coordinate system to NED
        """
        # Use the coordinate system conversion function
        ned_velocity = convert_velocity_to_ned(msg.linear, -self.trueYaw, self._coordinate_system)
        self.velocity = ned_velocity
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        """Extract current yaw angle from quaternion orientation"""
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    def cmdloop_callback(self):
        """
        Main control loop:
        1. Publish offboard control mode
        2. Send trajectory setpoints
        """
        if(self.offboardMode == True):
            # Publish heartbeat / offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            self.publisher_offboard_mode.publish(offboard_msg)            

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = float(self.velocity.x)
            trajectory_msg.velocity[1] = float(self.velocity.y)
            trajectory_msg.velocity[2] = float(self.velocity.z)
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = float('nan')
            trajectory_msg.acceleration[0] = float('nan')
            trajectory_msg.acceleration[1] = float('nan')
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = float('nan')
            trajectory_msg.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

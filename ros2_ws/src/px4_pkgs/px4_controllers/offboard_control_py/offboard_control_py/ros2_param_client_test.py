#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import sys
import argparse

class ParameterClientTester(Node):
    """Test node for setting PX4 parameters via ROS2 parameter service."""

    def __init__(self):
        super().__init__('param_client_tester')
        self._param_clients = {}
        
    def set_parameter(self, namespace, param_id, param_value):
        """Set a PX4 parameter using ROS2 parameter client API."""
        # Create the parameter service name
        service_name = f"{namespace}/param/set_parameters"
        
        # Create or reuse a client for the parameter service
        if service_name not in self._param_clients:
            self.get_logger().info(f'Creating new parameter client for {service_name}')
            self._param_clients[service_name] = self.create_client(
                SetParameters, 
                service_name
            )
        
        param_client = self._param_clients[service_name]
        
        # Wait for the service to be available
        self.get_logger().info(f'Waiting for parameter service {service_name}...')
        if not param_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'Parameter service {service_name} not available after timeout')
            return False
        
        # Create parameter value - FIXED: Use ParameterType instead of ParameterValue for type constant
        param_value_msg = ParameterValue()
        param_value_msg.type = ParameterType.PARAMETER_DOUBLE
        param_value_msg.double_value = float(param_value)
        
        # Create parameter
        param = Parameter()
        param.name = param_id
        param.value = param_value_msg
        
        # Create the request
        request = SetParameters.Request()
        request.parameters = [param]
        
        # Send the request
        self.get_logger().info(f'Sending parameter set request: {param_id}={param_value}')
        future = param_client.call_async(request)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        
        # Check the result
        if future.result() is not None:
            result = future.result()
            if len(result.results) > 0 and result.results[0].successful:
                self.get_logger().info(f'SUCCESS: Set param {param_id} to {param_value}')
                return True
            else:
                reason = result.results[0].reason if len(result.results) > 0 else "Unknown error"
                self.get_logger().error(f'FAILED: Unable to set param {param_id}: {reason}')
                return False
        else:
            self.get_logger().error(f'FAILED: Service call failed for param {param_id}')
            return False

def main(args=None):
    parser = argparse.ArgumentParser(description='Set PX4 parameters via ROS2')
    parser.add_argument('--namespace', type=str, required=True, help='PX4 namespace (e.g., /px4_0)')
    parser.add_argument('--param', type=str, required=True, help='Parameter ID to set')
    parser.add_argument('--value', type=float, required=True, help='Parameter value')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    param_client = ParameterClientTester()
    
    try:
        result = param_client.set_parameter(
            parsed_args.namespace,
            parsed_args.param,
            parsed_args.value
        )
        
        if not result:
            sys.exit(1)
    finally:
        param_client.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
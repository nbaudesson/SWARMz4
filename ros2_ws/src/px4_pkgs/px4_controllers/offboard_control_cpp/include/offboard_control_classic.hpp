#ifndef OFFBOARD_CONTROL_CLASSIC_HPP_
#define OFFBOARD_CONTROL_CLASSIC_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

/**
 * @brief Basic hover example for PX4 offboard control
 * 
 * A minimal implementation demonstrating:
 * - Basic publisher/subscriber setup
 * - Arming sequence
 * - Simple hover command
 * 
 * Example usage:
 * ```bash
 * ros2 run offboard_control_cpp offboard_control_classic --ros-args -r __ns:=/px4_1
 * ```
 */
class OffboardControlClassic : public rclcpp::Node {
public:
    explicit OffboardControlClassic();

private:
    // Node configuration
    std::string node_namespace_;  ///< ROS namespace (e.g., /px4_1)
    int instance_;               ///< PX4 instance number (0-based)
    
    // ROS interfaces
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State tracking
    uint64_t offboard_setpoint_counter_{0};  ///< Counts setpoints for initialization
    
    // Control methods
    /**
     * @brief Main control loop running at 10Hz
     * 
     * Sequence:
     * 1. First 10 cycles: Publish setpoints
     * 2. At 10 cycles: Arm and switch to offboard
     * 3. Maintain hover at 5m altitude
     */
    void timer_callback();             // Main control loop
    void publish_offboard_control_mode();  // Required for offboard control
    void publish_trajectory_setpoint();    // Position commands
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void arm();    // Send arm command
    void disarm(); // Send disarm command
};

#endif  // OFFBOARD_CONTROL_CLASSIC_HPP_

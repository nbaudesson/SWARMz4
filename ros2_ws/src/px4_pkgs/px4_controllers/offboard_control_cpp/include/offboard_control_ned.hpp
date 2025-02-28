#ifndef OFFBOARD_CONTROL_NED_HPP_
#define OFFBOARD_CONTROL_NED_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <array>

using namespace std::chrono_literals;

/**
 * @brief North-East-Down frame controller for PX4
 * 
 * Handles position control in the NED frame with:
 * - Initial heading compensation
 * - Frame transformations (NED to local)
 * - Position target tracking
 * - Auto takeoff and landing
 */
class OffboardControlNED : public rclcpp::Node {
public:
    explicit OffboardControlNED();

private:
    // Flight parameters
    static constexpr double TAKEOFF_HEIGHT = 2.0;        // Target takeoff altitude
    static constexpr double POSITION_THRESHOLD = 0.3;    // Distance to consider position reached
    static constexpr double TAKEOFF_THRESHOLD = 0.1;     // Altitude error for takeoff completion
    static constexpr double LANDING_THRESHOLD = 0.3;     // Distance from ground for landing
    static constexpr double POSE_TIMEOUT = 20.0;         // Time before auto-landing

    // Navigation state
    double initial_heading_;           // Initial heading in radians
    std::string node_namespace_;       // ROS namespace for multi-drone
    int instance_;                     // Instance number from namespace
    std::array<double, 3> initial_position_;  // Starting position
    bool ready_{false};               // Initial position received
    bool armed_{false};               // Vehicle armed state
    bool in_offboard_mode_{false};    // Offboard control active
    bool has_taken_off_{false};       // Takeoff complete
    bool objective_complete_{false};   // Target reached
    double last_pose_time_{0.0};      // Last target update time

    // Add current position tracking
    std::array<double, 3> current_position_{0.0, 0.0, 0.0};  // Track current position separately
    
    // Add debug counters
    int debug_counter_{0};
    int command_retry_counter_{0};
    
    // Add FCU ready flags
    bool fcu_params_ready_{false};
    int initialization_retry_counter_{0};
    bool preflight_checks_ok_{false};

    // Add initialization timer
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Add landing state flag
    bool landing_commanded_{false};  // Track if land command was sent

    // ROS interfaces
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Pose::SharedPtr target_pose_;

    // Control methods
    void timer_callback();
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    bool check_position_reached();
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
    std::array<double, 3> ned_to_local(const std::array<double, 3>& ned_pos);
    double ned_to_local_yaw(double ned_yaw);
    void publish_offboard_control_mode();
    void publish_position_setpoint(double x, double y, double z, double yaw = 0.0);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void arm();
    void disarm();
    void engage_offboard_mode();
    void land();

    void initialization_check();   // FCU connection checker
    void preflight_checks();      // Vehicle status checker

    // Store vehicle local position for preflight checks
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position;
    px4_msgs::msg::VehicleStatus vehicle_status;  // Add vehicle status storage
};

#endif  // OFFBOARD_CONTROL_NED_HPP_

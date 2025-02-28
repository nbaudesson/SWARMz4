#ifndef OFFBOARD_CONTROL_GOTO_HPP_
#define OFFBOARD_CONTROL_GOTO_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <array>
#include <vector>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

/**
 * @brief Waypoint structure to hold target position and timing information
 * 
 * All coordinates are in meters relative to the initial position
 * - x: Forward/North direction
 * - y: Right/East direction
 * - z: Down direction (negative = up)
 * - yaw: Heading in degrees (0 = forward/north)
 * - hover_time: How long to hold position in seconds
 */
struct Waypoint {
    double x;            // Forward/North position (m)
    double y;            // Right/East position (m)
    double z;            // Down position (m)
    double yaw;         // Heading (degrees)
    double hover_time;  // Hold time (seconds)
};

/**
 * @brief Waypoint navigation controller for autonomous missions
 * 
 * Features:
 * - YAML mission file loading
 * - Multi-drone support via namespaces
 * - Frame transformations (NED/FRD)
 * - Automatic takeoff, waypoint following, and landing
 */
class OffboardControlGoto : public rclcpp::Node {
public:
    explicit OffboardControlGoto();

private:
    // Parameters
    static constexpr double TAKEOFF_HEIGHT = 2.0;        // meters
    static constexpr double POSITION_THRESHOLD = 0.3;    // meters
    static constexpr double TAKEOFF_THRESHOLD = 0.1;     // meters

    // Node state
    std::string node_namespace_;
    int instance_;
    std::array<double, 3> initial_position_{0.0, 0.0, 0.0};
    std::array<double, 3> current_position_{0.0, 0.0, 0.0};
    std::array<double, 3> target_position_{0.0, 0.0, 0.0};
    bool ready_{false};
    bool armed_{false};
    bool in_offboard_mode_{false};
    bool has_taken_off_{false};
    bool objective_complete_{false};
    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_{0};
    double hover_start_time_{0.0};
    std::string frame_;  // 'ned' or 'frd'
    bool position_reached_logged_{false};  // Add this line

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Methods
    void load_waypoints(const std::string& mission_file);
    std::vector<Waypoint> get_default_waypoints();
    void timer_callback();
    void initialization_check();
    void preflight_checks();
    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    bool check_position_reached(const std::array<double, 3>& target_pos);
    bool check_hover_complete(double hover_time);
    std::tuple<std::array<double, 3>, double, double> get_next_setpoint();
    std::tuple<std::array<double, 3>, double> transform_waypoint(const Waypoint& waypoint);
    void publish_offboard_control_mode();
    void publish_position_setpoint(double x, double y, double z, double yaw = 0.0);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void arm();
    void disarm();
    void engage_offboard_mode();
    void land();
};

#endif  // OFFBOARD_CONTROL_GOTO_HPP_

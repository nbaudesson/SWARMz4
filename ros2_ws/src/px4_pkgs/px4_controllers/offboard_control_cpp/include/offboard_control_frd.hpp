#ifndef OFFBOARD_CONTROL_FRD_HPP_
#define OFFBOARD_CONTROL_FRD_HPP_

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
 * @brief Forward-Right-Down frame controller for PX4
 * 
 * Controls position in body-relative coordinates:
 * - Forward: Drone's forward direction (+X)
 * - Right: Drone's right side (+Y)
 * - Down: Towards ground (+Z)
 * 
 * Features:
 * - Body-relative position commands
 * - Automatic takeoff sequence
 * - Position hold with timeout
 * - Auto-land when idle
 * 
 * Example usage:
 * ```bash
 * ros2 run offboard_control_cpp offboard_control_frd --ros-args -r __ns:=/px4_1
 * ros2 topic pub /px4_1/target_pose geometry_msgs/msg/Pose "{position: {x: 5.0}}"
 * ```
 */
class OffboardControlFRD : public rclcpp::Node {
public:
    explicit OffboardControlFRD();

private:
    // Flight parameters
    static constexpr double TAKEOFF_HEIGHT = 2.0;     ///< Initial takeoff altitude
    static constexpr double POSITION_THRESHOLD = 0.3;  ///< Distance considered "reached"
    static constexpr double TAKEOFF_THRESHOLD = 0.1;   ///< Takeoff completion precision
    static constexpr double LANDING_THRESHOLD = 0.3;   ///< Landing detection height
    static constexpr double POSE_TIMEOUT = 20.0;       ///< Seconds before auto-land

    // System state
    std::string node_namespace_;        // For multi-drone support
    int instance_;                      // Vehicle instance number
    std::array<double, 3> initial_position_;  // Starting position reference
    bool ready_{false};                 // Initial position acquired
    bool armed_{false};                 // Vehicle is armed
    bool in_offboard_mode_{false};      // Offboard control active
    bool has_taken_off_{false};         // Takeoff sequence complete
    bool objective_complete_{false};     // Target position reached
    double last_pose_time_{0.0};        // Last target update time

    // Add initialization and state flags
    bool fcu_params_ready_{false};
    int initialization_retry_counter_{0};
    bool preflight_checks_ok_{false};
    bool landing_commanded_{false};
    bool position_reached_logged_{false};

    // Add initialization timer
    rclcpp::TimerBase::SharedPtr init_timer_;

    // Store vehicle status
    px4_msgs::msg::VehicleLocalPosition vehicle_local_position;
    px4_msgs::msg::VehicleStatus vehicle_status;

    // Add current position tracking
    std::array<double, 3> current_position_{0.0, 0.0, 0.0};  // Track current position separately

    // Add debug and retry counters
    int debug_counter_{0};
    int command_retry_counter_{0};

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
    /**
     * @brief Main control loop at 10Hz
     * 
     * Handles state machine:
     * 1. Wait for initial position
     * 2. Arm and enable offboard mode
     * 3. Execute takeoff sequence
     * 4. Track position targets
     * 5. Auto-land on timeout
     */
    void timer_callback();

    void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    
    /**
     * @brief Process position target updates
     * @param msg Target pose in FRD coordinates
     * 
     * Accepts position targets relative to current position:
     * - position.x: Forward distance (meters)
     * - position.y: Right distance (meters)
     * - position.z: Down distance (meters, negative = up)
     * - orientation: Target yaw as quaternion
     */
    void target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    bool check_position_reached();
    double quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q);
    void publish_offboard_control_mode();
    void publish_position_setpoint(double x, double y, double z, double yaw = 0.0);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void arm();
    void disarm();
    void engage_offboard_mode();
    void land();

    // Add new methods
    void initialization_check();
    void preflight_checks();
};

#endif  // OFFBOARD_CONTROL_FRD_HPP_

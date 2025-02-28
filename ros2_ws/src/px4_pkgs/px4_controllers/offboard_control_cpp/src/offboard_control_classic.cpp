/**
 * @brief Basic PX4 Offboard Control Demo
 * 
 * A minimal implementation demonstrating the essential requirements
 * for PX4 offboard control:
 * 1. Offboard control mode publishing
 * 2. Trajectory setpoint publishing
 * 3. Vehicle command handling
 * 
 * Features:
 * - Simple hover at 5m altitude
 * - Basic arming sequence
 * - Multi-drone support via namespaces
 * 
 * Usage:
 * ```bash
 * # Basic hover test
 * ros2 run offboard_control_cpp offboard_control_classic --ros-args -r __ns:=/px4_1
 * ```
 * 
 * Control Flow:
 * 1. First second (10 cycles): Establish comm with vehicle
 * 2. After 10 cycles: Send arm and offboard commands
 * 3. Continuous: Maintain hover at 5m altitude
 */

#include "offboard_control_classic.hpp"

/**
 * @brief Initialize publishers and timer for control loop
 * 
 * Sets up:
 * - Required PX4 publishers
 * - 10Hz control loop timer
 * - Multi-vehicle namespace handling
 */
OffboardControlClassic::OffboardControlClassic() : Node("offboard_control_classic") {
    // Get namespace
    node_namespace_ = this->get_namespace();
    
    // Extract instance number
    size_t last_slash = node_namespace_.rfind('_');
    if (last_slash != std::string::npos && last_slash + 1 < node_namespace_.length()) {
        try {
            instance_ = std::stoi(node_namespace_.substr(last_slash + 1));
        } catch (...) {
            instance_ = 0;
        }
    } else {
        instance_ = 0;
    }

    // Create publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        node_namespace_ + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        node_namespace_ + "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        node_namespace_ + "/fmu/in/vehicle_command", 10);

    // Create timer
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlClassic::timer_callback, this));
}

/**
 * @brief Main control loop running at 10Hz
 * 
 * State Machine:
 * 1. Initialization (first 10 cycles)
 *    - Publish setpoints to establish offboard control
 * 
 * 2. Mode Switch (at cycle 10)
 *    - Switch to offboard mode
 *    - Send arm command
 * 
 * 3. Normal Operation (after cycle 10)
 *    - Maintain position at 5m height
 *    - Continue publishing required messages
 */
void OffboardControlClassic::timer_callback() {
    if (offboard_setpoint_counter_ == 10) {
        // Change to offboard mode after 10 setpoints
        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        // Arm the vehicle
        this->arm();
    }

    // Send offboard control modes and setpoints
    this->publish_offboard_control_mode();
    this->publish_trajectory_setpoint();

    // Stop counter increment after 11
    if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
    }
}

/**
 * @brief Required offboard control heartbeat
 * 
 * PX4 requires this message at >2Hz to maintain offboard mode.
 * Configures which control dimensions are active:
 * - position: true (we want position control)
 * - others: false (velocity, acceleration, attitude, rates not used)
 */
void OffboardControlClassic::publish_offboard_control_mode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish position setpoint for hover
 * 
 * Sets target position in local frame:
 * - x: 0.0m (maintain horizontal position)
 * - y: 0.0m (maintain horizontal position)
 * - z: -5.0m (5 meters above start point)
 * - yaw: -π rad (facing original direction)
 */
void OffboardControlClassic::publish_trajectory_setpoint() {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.position = {0.0, 0.0, -5.0};  // Hover at 5 meters
    msg.yaw = -3.14;  // -PI
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControlClassic::publish_vehicle_command(uint16_t command, float param1, float param2) {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1 + instance_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControlClassic::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlClassic::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control classic node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlClassic>());
    rclcpp::shutdown();
    return 0;
}

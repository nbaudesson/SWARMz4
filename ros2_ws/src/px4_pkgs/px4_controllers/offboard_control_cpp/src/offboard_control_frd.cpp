/**
 * @brief Forward-Right-Down Frame Position Controller
 * 
 * This controller provides intuitive control in the drone's body frame:
 * - Forward: Along drone's forward direction (+X)
 * - Right: Along drone's right side (+Y)
 * - Down: Towards the ground (+Z)
 * 
 * Features:
 * - Body-relative position commands
 * - Automatic takeoff sequence
 * - Position hold with timeout
 * - Auto-landing on idle
 * 
 * Example Usage:
 * ```bash
 * # Start the controller
 * ros2 run offboard_control_cpp offboard_control_frd --ros-args -r __ns:=/px4_1
 * 
 * # Move 5m forward and 2m up
 * ros2 topic pub --once /px4_1/target_pose geometry_msgs/msg/Pose "{position: {x: 5.0, y: 0.0, z: -2.0}}"
 * ```
 */

#include "offboard_control_frd.hpp"
#include <cmath>

OffboardControlFRD::OffboardControlFRD() : Node("offboard_control_frd") {
    // Get namespace and extract instance
    node_namespace_ = this->get_namespace();
    
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

    // QoS profile for PX4 communication
    auto qos_profile = rclcpp::QoS(10)
        .best_effort()
        .durability_volatile()
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Create publishers
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        node_namespace_ + "/fmu/in/offboard_control_mode", qos_profile);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        node_namespace_ + "/fmu/in/trajectory_setpoint", qos_profile);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        node_namespace_ + "/fmu/in/vehicle_command", qos_profile);

    // Create subscribers
    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        node_namespace_ + "/fmu/out/vehicle_local_position",
        qos_profile,
        std::bind(&OffboardControlFRD::local_position_callback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        node_namespace_ + "/fmu/out/vehicle_status",
        qos_profile,
        std::bind(&OffboardControlFRD::vehicle_status_callback, this, std::placeholders::_1));
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        node_namespace_ + "/target_pose", 10,
        std::bind(&OffboardControlFRD::target_pose_callback, this, std::placeholders::_1));

    // Create control loop timer
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlFRD::timer_callback, this));

    // Add initialization timer before main timer
    init_timer_ = this->create_wall_timer(1s, std::bind(&OffboardControlFRD::initialization_check, this));

    RCLCPP_INFO(this->get_logger(), "FRD Controller initialized");
}

void OffboardControlFRD::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // Store full message
    vehicle_local_position = *msg;
    
    if (!msg->xy_valid) return;
    
    // Update current position
    current_position_ = {msg->x, msg->y, msg->z};
    
    // Set initial position only once
    if (!ready_) {
        initial_position_ = current_position_;
        ready_ = true;
        RCLCPP_INFO(this->get_logger(), 
            "Initial position set: [%.2f, %.2f, %.2f]",
            msg->x, msg->y, msg->z);
    }
}

void OffboardControlFRD::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    // Store full message
    vehicle_status = *msg;
    
    bool prev_armed = armed_;
    bool prev_offboard = in_offboard_mode_;

    armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    in_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // Reset landing flag if disarmed
    if (prev_armed && !armed_) {
        landing_commanded_ = false;
        RCLCPP_INFO(this->get_logger(), "Landing complete, vehicle disarmed");
    }

    if (prev_armed != armed_) {
        RCLCPP_INFO(this->get_logger(), "Armed state changed to: %s", armed_ ? "ARMED" : "DISARMED");
    }
    if (prev_offboard != in_offboard_mode_) {
        RCLCPP_INFO(this->get_logger(), "Offboard mode changed to: %s", in_offboard_mode_ ? "ENABLED" : "DISABLED");
    }
}

/**
 * @brief Process new target poses in FRD frame
 * 
 * Accepts position commands relative to current position:
 * - x: Forward distance (meters)
 * - y: Right distance (meters)
 * - z: Down distance (meters, negative = up)
 * - orientation: Desired yaw as quaternion
 * 
 * Example:
 * - position(5,0,0): Move 5m forward
 * - position(0,5,0): Move 5m right
 * - position(0,0,-2): Move 2m up
 */
void OffboardControlFRD::target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!ready_) {
        RCLCPP_WARN(this->get_logger(), "Not ready: No initial position yet");
        return;
    }

    target_pose_ = msg;
    last_pose_time_ = this->now().seconds();
    objective_complete_ = false;

    RCLCPP_INFO(this->get_logger(),
        "New target pose: [%.1f, %.1f, %.1f]",
        msg->position.x, msg->position.y, msg->position.z);
}

/**
 * @brief Check if target position has been reached
 * 
 * Uses 3D Euclidean distance with POSITION_THRESHOLD:
 * - Compares current position to target
 * - Returns true when within threshold
 * - Accounts for initial position offset
 */
bool OffboardControlFRD::check_position_reached() {
    if (!target_pose_) return false;

    // Calculate target position in local frame
    double target_x = initial_position_[0] + target_pose_->position.x;
    double target_y = initial_position_[1] + target_pose_->position.y;
    double target_z = initial_position_[2] + target_pose_->position.z;

    // Compare with current position
    double dx = target_x - current_position_[0];
    double dy = target_y - current_position_[1];
    double dz = target_z - current_position_[2];

    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (distance < POSITION_THRESHOLD) {
        if (!objective_complete_) {
            last_pose_time_ = this->now().seconds();
            RCLCPP_INFO(this->get_logger(), 
                "Position reached with error: %.2f m, starting %.1f second timeout",
                distance, POSE_TIMEOUT);
        }
        return true;
    }
    return false;
}

double OffboardControlFRD::quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to yaw angle
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Main control loop running at 10Hz
 * 
 * State Machine:
 * 1. Setup Phase
 *    - Get initial position
 *    - Arm vehicle
 *    - Enter offboard mode
 * 
 * 2. Takeoff Phase
 *    - Rise to TAKEOFF_HEIGHT
 *    - Hold position until height reached
 * 
 * 3. Navigation Phase
 *    - Track FRD position targets
 *    - Hold position when no target
 *    - Auto-land after POSE_TIMEOUT seconds of inactivity
 */
void OffboardControlFRD::timer_callback() {
    publish_offboard_control_mode();  // Fixed function call

    // Debug info (add this to track timing)
    debug_counter_++;
    if (debug_counter_ >= 50) {
        auto current_time = this->now().seconds();
        RCLCPP_INFO(this->get_logger(),
            "Status - Armed: %s, Offboard: %s, Complete: %s, Time since target: %.1f, "
            "Position: [%.1f, %.1f, %.1f]",
            armed_ ? "true" : "false",
            in_offboard_mode_ ? "true" : "false",
            objective_complete_ ? "true" : "false",
            current_time - last_pose_time_,
            current_position_[0], current_position_[1], current_position_[2]);
        debug_counter_ = 0;
    }

    // Check initialization and preflight checks
    if (!fcu_params_ready_ || !preflight_checks_ok_ || !ready_) {
        return;
    }

    auto current_time = this->now().seconds();

    // Check for landing conditions
    if (!landing_commanded_ && 
        objective_complete_ && 
        last_pose_time_ > 0 && 
        target_pose_ && 
        current_time - last_pose_time_ > POSE_TIMEOUT) 
    {
        RCLCPP_INFO(this->get_logger(), 
            "Landing sequence initiated after %.1f seconds at target",
            current_time - last_pose_time_);
        land();
        landing_commanded_ = true;
        return;
    }

    // If we're landing, don't continue with position control
    if (landing_commanded_) {
        return;
    }

    // State machine for arming/offboard - only when we have a target
    if (!armed_ || !in_offboard_mode_) {
        command_retry_counter_++;
        if (command_retry_counter_ >= 10) {
            command_retry_counter_ = 0;
            if (!armed_) {
                arm();
                RCLCPP_INFO(this->get_logger(), "Retrying arm command...");
            }
            if (!in_offboard_mode_) {
                engage_offboard_mode();
                RCLCPP_INFO(this->get_logger(), "Retrying offboard mode...");
            }
        }
        return;
    }

    // Navigation logic
    if (!has_taken_off_) {
        // Takeoff sequence
        double takeoff_z = initial_position_[2] - TAKEOFF_HEIGHT;
        publish_position_setpoint(
            initial_position_[0],
            initial_position_[1],
            takeoff_z
        );
        
        if (std::abs(current_position_[2] - takeoff_z) < TAKEOFF_THRESHOLD) {
            has_taken_off_ = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff complete");
        }
    } else if (target_pose_) {
        // Calculate target in local frame (FRD coordinates relative to initial position)
        double target_x = initial_position_[0] + target_pose_->position.x;
        double target_y = initial_position_[1] + target_pose_->position.y;
        double target_z = initial_position_[2] + target_pose_->position.z;
        double yaw = quaternion_to_yaw(target_pose_->orientation);

        // Always publish target position for consistent control
        publish_position_setpoint(target_x, target_y, target_z, yaw);
        
        // Check if reached target (only for status update)
        if (!objective_complete_ && check_position_reached()) {
            objective_complete_ = true;
            last_pose_time_ = this->now().seconds();  // Start timeout when reaching target
            RCLCPP_INFO(this->get_logger(), "Target position reached!");
        }
    } else {
        // Hold current position when no target
        publish_position_setpoint(
            current_position_[0],
            current_position_[1],
            current_position_[2]
        );
    }
}

void OffboardControlFRD::publish_offboard_control_mode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControlFRD::publish_position_setpoint(double x, double y, double z, double yaw) {
    auto msg = px4_msgs::msg::TrajectorySetpoint();
    msg.position = {
        static_cast<float>(x),
        static_cast<float>(y),
        static_cast<float>(z)
    };
    msg.yaw = static_cast<float>(yaw);
    msg.timestamp = this->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControlFRD::publish_vehicle_command(uint16_t command, float param1, float param2) {
    auto msg = px4_msgs::msg::VehicleCommand();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1 + instance_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControlFRD::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlFRD::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControlFRD::engage_offboard_mode() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
}

void OffboardControlFRD::land() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Switching to land mode");
}

void OffboardControlFRD::initialization_check() {
    if (!fcu_params_ready_) {
        initialization_retry_counter_++;
        RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
        
        if (vehicle_local_position.xy_valid && 
            vehicle_local_position.timestamp > 0) {
            fcu_params_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "FCU connection established!");
            preflight_checks();
            init_timer_->cancel();
        } else if (initialization_retry_counter_ > 10) {
            RCLCPP_ERROR(this->get_logger(), "Failed to establish FCU connection!");
            rclcpp::shutdown();
        }
    }
}

void OffboardControlFRD::preflight_checks() {
    bool checks_passed = true;
    
    if (!vehicle_local_position.xy_valid) {
        RCLCPP_WARN(this->get_logger(), "Position data not valid");
        checks_passed = false;
    }
    
    if (vehicle_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
        vehicle_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_WARN(this->get_logger(), "Vehicle in invalid arming state");
        checks_passed = false;
    }
    
    if (vehicle_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_TERMINATION) {
        RCLCPP_WARN(this->get_logger(), "Vehicle in critical state");
        checks_passed = false;
    }
    
    if (checks_passed) {
        preflight_checks_ok_ = true;
        RCLCPP_INFO(this->get_logger(), "Pre-flight checks passed");
    } else {
        RCLCPP_WARN(this->get_logger(), "Pre-flight checks failed, retrying...");
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control FRD node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlFRD>());
    rclcpp::shutdown();
    return 0;
}

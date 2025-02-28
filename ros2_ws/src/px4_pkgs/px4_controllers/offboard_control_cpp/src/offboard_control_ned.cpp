/**
 * @brief PX4 Position Control in North-East-Down Frame
 * 
 * This controller provides position control in the NED frame, which is useful for:
 * - Multi-vehicle coordination using absolute positions
 * - GPS-relative navigation
 * - Ground control station integration
 * 
 * Features:
 * - NED frame position commands (North=+X, East=+Y, Down=+Z)
 * - Initial heading compensation
 * - Automatic takeoff and landing
 * - Position hold with timeout
 * 
 * Usage:
 * ```bash
 * # Start with 45° initial heading
 * ros2 run offboard_control_cpp offboard_control_ned --ros-args -r __ns:=/px4_1 -p initial_heading:=45.0
 * 
 * # Send NED position command
 * ros2 topic pub --once /px4_1/target_ned_pose geometry_msgs/msg/Pose "{position: {x: 10.0, y: 0.0, z: -5.0}}"  # 10m North, 5m altitude
 * ```
 */

#include "offboard_control_ned.hpp"
#include <cmath>

OffboardControlNED::OffboardControlNED() : Node("offboard_control_ned") {
    // Declare and get parameters
    this->declare_parameter("initial_heading", 0.0);
    initial_heading_ = this->get_parameter("initial_heading").as_double() * M_PI / 180.0;  // Convert to radians

    // Get namespace and extract instance (same as FRD)
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

    // Setup QoS profile and create publishers/subscribers (same as FRD)
    auto qos_profile = rclcpp::QoS(10)
        .best_effort()
        .durability_volatile()
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Create publishers and subscribers (same as FRD)
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        node_namespace_ + "/fmu/in/offboard_control_mode", qos_profile);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        node_namespace_ + "/fmu/in/trajectory_setpoint", qos_profile);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        node_namespace_ + "/fmu/in/vehicle_command", qos_profile);

    local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        node_namespace_ + "/fmu/out/vehicle_local_position",
        qos_profile,
        std::bind(&OffboardControlNED::local_position_callback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        node_namespace_ + "/fmu/out/vehicle_status",
        qos_profile,
        std::bind(&OffboardControlNED::vehicle_status_callback, this, std::placeholders::_1));
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        node_namespace_ + "/target_ned_pose", 10,
        std::bind(&OffboardControlNED::target_pose_callback, this, std::placeholders::_1));

    // Create initialization timer (note: add this before the main timer)
    init_timer_ = this->create_wall_timer(1s, std::bind(&OffboardControlNED::initialization_check, this));

    // Regular control timer
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlNED::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), 
        "NED Controller initialized with heading: %.1f degrees", 
        initial_heading_ * 180.0 / M_PI);
}

/**
 * @brief Transform NED coordinates to local frame
 * 
 * Converts North-East-Down coordinates to the drone's local frame
 * using the initial heading rotation:
 * 
 * [x_local]   [cos(h) -sin(h)] [north]
 * [y_local] = [sin(h)  cos(h)] [east]
 * 
 * Where h is the initial heading angle
 */
std::array<double, 3> OffboardControlNED::ned_to_local(const std::array<double, 3>& ned_pos) {
    // Convert NED position to local frame using initial heading rotation
    double c = std::cos(initial_heading_);
    double s = std::sin(initial_heading_);
    
    std::array<double, 3> local_pos;
    local_pos[0] = c * ned_pos[0] - s * ned_pos[1] + initial_position_[0];
    local_pos[1] = s * ned_pos[0] + c * ned_pos[1] + initial_position_[1];
    local_pos[2] = ned_pos[2] + initial_position_[2];
    
    return local_pos;
}

double OffboardControlNED::ned_to_local_yaw(double ned_yaw) {
    // Convert NED yaw to local frame
    return ned_yaw + initial_heading_;
}

/**
 * @brief Process target poses in NED frame
 * 
 * Handles incoming pose commands in NED frame:
 * - position: NED coordinates in meters
 * - orientation: heading as quaternion (yaw from North)
 * 
 * Example:
 * - position(10,0,0): 10 meters North
 * - position(0,10,0): 10 meters East
 * - position(0,0,-5): 5 meters Up
 */
void OffboardControlNED::target_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    if (!ready_) {
        RCLCPP_WARN(this->get_logger(), "Not ready: No initial position yet");
        return;
    }

    // Convert NED position to local frame
    std::array<double, 3> ned_pos = {
        msg->position.x,
        msg->position.y,
        msg->position.z
    };
    double ned_yaw = 2 * std::atan2(msg->orientation.z, msg->orientation.w);

    auto local_pos = ned_to_local(ned_pos);
    double local_yaw = ned_to_local_yaw(ned_yaw);

    // Create new pose message in local frame
    auto local_pose = std::make_shared<geometry_msgs::msg::Pose>();
    local_pose->position.x = local_pos[0];
    local_pose->position.y = local_pos[1];
    local_pose->position.z = local_pos[2];
    
    // Convert yaw to quaternion
    local_pose->orientation.w = std::cos(local_yaw / 2);
    local_pose->orientation.x = 0.0;
    local_pose->orientation.y = 0.0;
    local_pose->orientation.z = std::sin(local_yaw / 2);

    target_pose_ = local_pose;
    objective_complete_ = false;  // Reset completion flag
    last_pose_time_ = this->now().seconds();  // Update time when receiving new target

    RCLCPP_INFO(this->get_logger(),
        "New target pose (NED -> Local): [%.1f, %.1f, %.1f] -> [%.1f, %.1f, %.1f]",
        ned_pos[0], ned_pos[1], ned_pos[2],
        local_pos[0], local_pos[1], local_pos[2]);
}

void OffboardControlNED::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // Store the full message
    vehicle_local_position = *msg;
    
    if (!msg->xy_valid) return;
    
    // Always update current position
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

// Update vehicle status callback to detect landing completion
void OffboardControlNED::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    // Store the full message
    vehicle_status = *msg;
    
    bool prev_armed = armed_;
    bool prev_offboard = in_offboard_mode_;

    armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    in_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    // Reset landing flag if we get disarmed (landing complete)
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
 * @brief Main control loop running at 10Hz
 * 
 * Control sequence:
 * 1. Initialization - Wait for valid position
 * 2. Takeoff Phase - Arm and reach target height
 * 3. Navigation Phase - Follow NED position targets
 */
void OffboardControlNED::timer_callback() {
    publish_offboard_control_mode();

    // Debug info
    debug_counter_++;
    if (debug_counter_ >= 50) {
        RCLCPP_INFO(this->get_logger(),
            "Status - Armed: %s, Offboard: %s, FCU ready: %s, Checks OK: %s, "
            "Position: [%.1f, %.1f, %.1f]",
            armed_ ? "true" : "false",
            in_offboard_mode_ ? "true" : "false",
            fcu_params_ready_ ? "true" : "false",
            preflight_checks_ok_ ? "true" : "false",
            current_position_[0], current_position_[1], current_position_[2]);
        debug_counter_ = 0;
    }

    // Check initialization and preflight checks
    if (!fcu_params_ready_ || !preflight_checks_ok_ || !ready_) {
        return;
    }

    auto current_time = this->now().seconds();

    // Check for landing conditions:
    // 1. Target is set and reached
    // 2. We've waited for POSE_TIMEOUT seconds after reaching target
    if (!landing_commanded_ &&  // Only check if not already landing
        objective_complete_ &&   // Must have reached target
        last_pose_time_ > 0 &&  // Must have received a pose
        target_pose_ &&         // Must have a target
        current_time - last_pose_time_ > POSE_TIMEOUT)  // Must have waited for timeout
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

    // Rest of control logic for normal operation
    if (!target_pose_) {
        // Hold current position if already flying
        if (armed_ && in_offboard_mode_ && has_taken_off_) {
            publish_position_setpoint(
                current_position_[0],
                current_position_[1],
                current_position_[2]
            );
        }
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

    // Navigation logic - only when armed and in offboard
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
    } else {
        // Regular navigation
        if (!objective_complete_) {
            publish_position_setpoint(
                target_pose_->position.x,
                target_pose_->position.y,
                target_pose_->position.z,
                2 * std::atan2(target_pose_->orientation.z, target_pose_->orientation.w)
            );

            if (check_position_reached()) {
                objective_complete_ = true;
                RCLCPP_INFO(this->get_logger(), "Target position reached!");
            }
        } else {
            // Hold position at target when objective complete
            publish_position_setpoint(
                target_pose_->position.x,
                target_pose_->position.y,
                target_pose_->position.z,
                2 * std::atan2(target_pose_->orientation.z, target_pose_->orientation.w)
            );
        }
    }
}

// Update initialization check with init timer cancellation
void OffboardControlNED::initialization_check() {
    if (!fcu_params_ready_) {
        initialization_retry_counter_++;
        RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
        
        // Check if we have valid position data
        if (vehicle_local_position.xy_valid && 
            vehicle_local_position.timestamp > 0) {
                
            fcu_params_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "FCU connection established!");
            preflight_checks();
            
            // Cancel init timer after successful initialization
            init_timer_->cancel();
        } else if (initialization_retry_counter_ > 10) {
            RCLCPP_ERROR(this->get_logger(), "Failed to establish FCU connection!");
            rclcpp::shutdown();
        }
    }
}

// Add preflight checks
void OffboardControlNED::preflight_checks() {
    bool checks_passed = true;
    
    // Check position data validity
    if (!vehicle_local_position.xy_valid) {
        RCLCPP_WARN(this->get_logger(), "Position data not valid");
        checks_passed = false;
    }
    
    // Check arming state validity
    if (vehicle_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
        vehicle_status.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_WARN(this->get_logger(), "Vehicle in invalid arming state");
        checks_passed = false;
    }
    
    // Check navigation state
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

/**
 * @brief Check if target position has been reached
 * 
 * Uses 3D Euclidean distance with POSITION_THRESHOLD:
 * - Compares current position to target
 * - Returns true when within threshold
 */
bool OffboardControlNED::check_position_reached() {
    if (!target_pose_) return false;

    // Compare current position with target position
    double dx = target_pose_->position.x - current_position_[0];  // Compare with current, not initial
    double dy = target_pose_->position.y - current_position_[1];
    double dz = target_pose_->position.z - current_position_[2];

    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (distance < POSITION_THRESHOLD) {
        if (!objective_complete_) {  // Only update time when first reaching target
            last_pose_time_ = this->now().seconds();  // Start timeout when reaching target
            RCLCPP_INFO(this->get_logger(), 
                "Position reached with error: %.2f m, starting %.1f second timeout",
                distance, POSE_TIMEOUT);
        }
        return true;
    }
    return false;
}

void OffboardControlNED::publish_offboard_control_mode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControlNED::publish_position_setpoint(double x, double y, double z, double yaw) {
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

void OffboardControlNED::publish_vehicle_command(uint16_t command, float param1, float param2) {
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

void OffboardControlNED::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlNED::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControlNED::engage_offboard_mode() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
}

void OffboardControlNED::land() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Switching to land mode");
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control NED node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlNED>());
    rclcpp::shutdown();
    return 0;
}
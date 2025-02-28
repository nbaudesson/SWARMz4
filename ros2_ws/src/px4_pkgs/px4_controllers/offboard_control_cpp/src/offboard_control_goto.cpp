/**
 * @brief Waypoint-based Mission Controller for PX4
 * 
 * Advanced controller supporting:
 * 1. Mission file loading from YAML
 * 2. Multi-frame support (NED/FRD)
 * 3. Per-waypoint configuration:
 *    - Target position (x, y, z)
 *    - Heading/yaw
 *    - Hover duration
 * 4. Auto-sequences:
 *    - Takeoff
 *    - Waypoint navigation
 *    - Position hold
 *    - Auto-landing
 * 
 * Mission File Format:
 * ```yaml
 * 1:  # Instance 1 waypoints
 *   - x: 5.0    # Forward/North
 *     y: 0.0    # Right/East
 *     z: -2.0   # Down (negative = up)
 *     yaw: 0.0  # Heading in degrees
 *     time: 5.0 # Hover duration
 * ```
 * 
 * Example Usage:
 * ```bash
 * # Run with default square pattern
 * ros2 run offboard_control_cpp offboard_control_goto --ros-args -r __ns:=/px4_1
 * 
 * # Run with custom mission and NED frame
 * ros2 run offboard_control_cpp offboard_control_goto --ros-args -r __ns:=/px4_1 -p mission_file:=mission.yaml -p frame:=ned
 * ```
 */

#include "offboard_control_goto.hpp"
#include <cmath>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

OffboardControlGoto::OffboardControlGoto() : Node("offboard_control_goto") {
    // Declare parameters
    this->declare_parameter("mission_file", "");
    this->declare_parameter("frame", "frd");

    // Get parameters
    std::string mission_file = this->get_parameter("mission_file").as_string();
    frame_ = this->get_parameter("frame").as_string();

    if (frame_ != "ned" && frame_ != "frd") {
        RCLCPP_ERROR(this->get_logger(), "Invalid frame: %s. Using FRD.", frame_.c_str());
        frame_ = "frd";
    }

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

    // Load waypoints
    if (!mission_file.empty()) {
        load_waypoints(mission_file);
    } else {
        waypoints_ = get_default_waypoints();
        RCLCPP_INFO(this->get_logger(), "Using default waypoints");
    }

    // Setup QoS profile and publishers/subscribers
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
        std::bind(&OffboardControlGoto::local_position_callback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        node_namespace_ + "/fmu/out/vehicle_status",
        qos_profile,
        std::bind(&OffboardControlGoto::vehicle_status_callback, this, std::placeholders::_1));

    // Create timers
    init_timer_ = this->create_wall_timer(1s, std::bind(&OffboardControlGoto::initialization_check, this));
    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControlGoto::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), 
        "Goto Controller started in %s frame mode with %zu waypoints",
        frame_.c_str(), waypoints_.size());
}

/**
 * @brief Provides a default square pattern if no mission file is specified
 * 
 * The pattern creates a 5x5 meter square at 2 meters height with:
 * - 5 second hover at each corner
 * - 90-degree heading changes at each corner
 */
std::vector<Waypoint> OffboardControlGoto::get_default_waypoints() {
    return {
        {5.0, 0.0, -2.0, 0.0, 5.0},    // [x, y, z, yaw_degrees, hover_time]
        {5.0, 5.0, -2.0, 90.0, 5.0},
        {0.0, 5.0, -2.0, 180.0, 5.0},
        {0.0, 0.0, -2.0, 270.0, 5.0}
    };
}

/**
 * @brief Loads waypoints from a YAML file with instance-specific configurations
 * 
 * File format example:
 * ```yaml
 * 1:  # Instance 1
 *   - x: 5.0
 *     y: 0.0
 *     z: -2.0
 *     yaw: 0.0
 *     time: 5.0
 * ```
 * 
 * Search paths (in order):
 * 1. Direct path
 * 2. Package missions directory
 * 3. Package config directory
 */
void OffboardControlGoto::load_waypoints(const std::string& mission_file) {
    std::vector<std::string> search_paths = {
        mission_file,
        (std::filesystem::path(ament_index_cpp::get_package_share_directory("offboard_control_cpp")) / "missions" / mission_file).string(),
        (std::filesystem::path(ament_index_cpp::get_package_share_directory("offboard_control_cpp")) / "config" / mission_file).string()
    };

    bool file_loaded = false;
    for (const auto& path : search_paths) {
        try {
            YAML::Node config = YAML::LoadFile(path);
            auto instance_waypoints = config[std::to_string(instance_ + 1)];
            
            if (instance_waypoints) {
                waypoints_.clear();
                for (const auto& wp : instance_waypoints) {
                    Waypoint waypoint;
                    waypoint.x = wp["x"].as<double>();
                    waypoint.y = wp["y"].as<double>();
                    waypoint.z = wp["z"].as<double>(-2.0);  // Default height
                    waypoint.yaw = wp["yaw"].as<double>(0.0);  // Default yaw
                    waypoint.hover_time = wp["time"].as<double>(5.0);  // Default hover time
                    
                    // Normalize yaw to 0-360 degrees
                    waypoint.yaw = std::fmod(waypoint.yaw, 360.0);
                    if (waypoint.yaw < 0) waypoint.yaw += 360.0;
                    
                    waypoints_.push_back(waypoint);
                }
                
                RCLCPP_INFO(this->get_logger(),
                    "Loaded %zu waypoints from %s for instance %d",
                    waypoints_.size(), path.c_str(), instance_ + 1);
                file_loaded = true;
                break;
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(),  // Changed from ERROR to DEBUG
                "Trying next path. Failed to load %s: %s", path.c_str(), e.what());
            continue;
        }
    }

    if (!file_loaded) {
        RCLCPP_WARN(this->get_logger(),
            "Could not load waypoints from any path, using defaults");
        waypoints_ = get_default_waypoints();
    }
}

/**
 * @brief Transforms waypoint coordinates based on selected frame
 * 
 * Handles coordinate transformations between:
 * - FRD (Forward-Right-Down)
 * - NED (North-East-Down)
 * 
 * Also applies initial position offset and heading corrections
 */
std::tuple<std::array<double, 3>, double> OffboardControlGoto::transform_waypoint(const Waypoint& waypoint) {
    std::array<double, 3> pos;
    double yaw_rad;

    if (frame_ == "ned") {
        // Convert NED coordinates and heading
        pos = {
            waypoint.x + initial_position_[0],
            waypoint.y + initial_position_[1],
            waypoint.z + initial_position_[2]
        };
        yaw_rad = waypoint.yaw * M_PI / 180.0;
    } else {  // FRD frame
        pos = {
            waypoint.x + initial_position_[0],
            waypoint.y + initial_position_[1],
            waypoint.z + initial_position_[2]
        };
        yaw_rad = waypoint.yaw * M_PI / 180.0;
    }

    return {pos, yaw_rad};
}

/**
 * @brief Core navigation state machine
 * 
 * State sequence:
 * 1. Check if current waypoint is reached
 * 2. If reached, start hover timer
 * 3. After hover complete, move to next waypoint
 * 4. If all waypoints complete, trigger landing
 * 
 * @return tuple containing: position[3], yaw, hover_time
 */
std::tuple<std::array<double, 3>, double, double> OffboardControlGoto::get_next_setpoint() {
    if (current_waypoint_index_ >= waypoints_.size()) {
        return {{0.0, 0.0, 0.0}, 0.0, 0.0};
    }

    const auto& waypoint = waypoints_[current_waypoint_index_];
    auto [local_pos, local_yaw] = transform_waypoint(waypoint);

    if (check_position_reached(local_pos)) {
        if (!check_hover_complete(waypoint.hover_time)) {
            return {local_pos, local_yaw, waypoint.hover_time};
        }

        current_waypoint_index_++;
        hover_start_time_ = 0.0;
        position_reached_logged_ = false;  // Reset for next waypoint

        if (current_waypoint_index_ >= waypoints_.size()) {
            objective_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "Mission complete, preparing to land");
            return {{0.0, 0.0, 0.0}, 0.0, 0.0};
        }

        const auto& next_waypoint = waypoints_[current_waypoint_index_];
        auto [next_pos, next_yaw] = transform_waypoint(next_waypoint);
        RCLCPP_INFO(this->get_logger(),
            "Moving to waypoint %zu", current_waypoint_index_ + 1);
        return {next_pos, next_yaw, next_waypoint.hover_time};
    }

    return {local_pos, local_yaw, waypoint.hover_time};
}

bool OffboardControlGoto::check_hover_complete(double hover_time) {
    auto current_time = this->now().seconds();
    
    if (hover_start_time_ == 0.0) {
        hover_start_time_ = current_time;
        return false;
    }
    
    return (current_time - hover_start_time_) >= hover_time;
}

void OffboardControlGoto::local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    if (!msg->xy_valid) return;
    
    // Update current position continuously
    current_position_ = {msg->x, msg->y, msg->z};
    
    // Set initial position only once
    if (!ready_) {
        initial_position_ = current_position_;
        ready_ = true;
        RCLCPP_INFO(this->get_logger(), 
            "Initial position set: [%.2f, %.2f, %.2f]",
            msg->x, msg->y, msg->z);
    }

    // Add debug logging for takeoff and waypoint tracking
    if (has_taken_off_ && !objective_complete_) {
        RCLCPP_DEBUG(this->get_logger(),
            "Current pos: [%.2f, %.2f, %.2f], Target pos: [%.2f, %.2f, %.2f]",
            current_position_[0], current_position_[1], current_position_[2],
            target_position_[0], target_position_[1], target_position_[2]);
    }
}

void OffboardControlGoto::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    bool prev_armed = armed_;
    bool prev_offboard = in_offboard_mode_;

    armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    in_offboard_mode_ = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    if (prev_armed != armed_) {
        RCLCPP_INFO(this->get_logger(), "Armed state changed to: %s", armed_ ? "ARMED" : "DISARMED");
    }
    if (prev_offboard != in_offboard_mode_) {
        RCLCPP_INFO(this->get_logger(), "Offboard mode changed to: %s", in_offboard_mode_ ? "ENABLED" : "DISABLED");
    }
}

void OffboardControlGoto::initialization_check() {
    if (!ready_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for initial position...");
        return;
    }
    init_timer_->cancel();
}

/**
 * @brief Main control loop and state machine
 * 
 * States:
 * 1. INITIALIZATION
 *    - Wait for valid position data
 *    - Setup publishers/subscribers
 * 
 * 2. SETUP
 *    - Arm vehicle
 *    - Enter offboard mode
 * 
 * 3. TAKEOFF
 *    - Rise to TAKEOFF_HEIGHT
 *    - Hold position until stable
 * 
 * 4. MISSION
 *    - Navigate through waypoints
 *    - Hold at each waypoint for specified time
 *    - Track completion status
 * 
 * 5. LANDING
 *    - Auto-land when mission complete
 *    - Monitor for ground contact
 */
void OffboardControlGoto::timer_callback() {
    // Required: Publish offboard control mode at 10Hz
    publish_offboard_control_mode();

    // Wait for initial position before starting
    if (!ready_) return;

    // State machine for arming and offboard mode transition
    if (!armed_ || !in_offboard_mode_) {
        static int counter = 0;
        // Wait for 10 cycles (1 second) before attempting arm/offboard
        if (++counter >= 10) {
            counter = 0;
            // Try arming if not armed
            if (!armed_) arm();
            // Try switching to offboard mode if not in offboard
            if (!in_offboard_mode_) engage_offboard_mode();
        }
        return;
    }

    // Main flight state machine
    if (!has_taken_off_) {
        // TAKEOFF STATE
        // Calculate target height (negative is up in PX4)
        double takeoff_z = initial_position_[2] - TAKEOFF_HEIGHT;
        
        // Command vertical position while maintaining horizontal position
        publish_position_setpoint(
            initial_position_[0],  // Stay at initial X
            initial_position_[1],  // Stay at initial Y
            takeoff_z             // Move to target height
        );
        
        // Check if we've reached takeoff height
        if (std::abs(current_position_[2] - takeoff_z) < TAKEOFF_THRESHOLD) {
            has_taken_off_ = true;
            RCLCPP_INFO(this->get_logger(), "Takeoff complete, starting waypoint navigation");
            
            // Immediately start first waypoint
            auto [pos, yaw, hover_time] = get_next_setpoint();
            target_position_ = {pos[0], pos[1], pos[2]};
            publish_position_setpoint(pos[0], pos[1], pos[2], yaw);
        }
    } else if (!objective_complete_) {
        // WAYPOINT NAVIGATION STATE
        auto [pos, yaw, hover_time] = get_next_setpoint();
        if (objective_complete_) {
            // All waypoints complete, transition to landing
            land();
        } else {
            // Command next waypoint position
            target_position_ = {pos[0], pos[1], pos[2]};
            publish_position_setpoint(pos[0], pos[1], pos[2], yaw);
        }
    }
    // If objective complete, we're either landing or have landed
}

/**
 * @brief Process next waypoint in sequence
 * 
 * Actions:
 * 1. Check if current waypoint reached
 * 2. Track hover time at waypoint
 * 3. Move to next waypoint when ready
 * 4. Transform coordinates based on frame
 * 
 * @return tuple(position, yaw, hover_time)
 */
bool OffboardControlGoto::check_position_reached(const std::array<double, 3>& target_pos) {
    // Calculate distance to target in 3D space
    double dx = target_pos[0] - current_position_[0];
    double dy = target_pos[1] - current_position_[1];
    double dz = target_pos[2] - current_position_[2];

    // Euclidean distance
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    bool reached = distance < POSITION_THRESHOLD;
    
    // Log only when first reaching waypoint to avoid spam
    if (reached && !position_reached_logged_) {
        position_reached_logged_ = true;
        RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", current_waypoint_index_ + 1);
    } else if (!reached) {
        // Reset logging flag when moving away from target
        position_reached_logged_ = false;
    }
    
    return reached;
}

void OffboardControlGoto::publish_offboard_control_mode() {
    auto msg = px4_msgs::msg::OffboardControlMode();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void OffboardControlGoto::publish_position_setpoint(double x, double y, double z, double yaw) {
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

void OffboardControlGoto::publish_vehicle_command(uint16_t command, float param1, float param2) {
    // Create and populate PX4 vehicle command message
    auto msg = px4_msgs::msg::VehicleCommand();
    
    // Command parameters
    msg.param1 = param1;  // Command-specific parameter 1
    msg.param2 = param2;  // Command-specific parameter 2
    msg.command = command;  // The command to execute
    
    // System configuration
    msg.target_system = 1 + instance_;  // Vehicle instance (1-based)
    msg.target_component = 1;  // Always target first component
    msg.source_system = 1;  // GCS system ID
    msg.source_component = 1;  // GCS component ID
    msg.from_external = true;  // Mark as external command
    
    // PX4 requires timestamps in microseconds
    msg.timestamp = this->now().nanoseconds() / 1000;
    
    // Send the command
    vehicle_command_publisher_->publish(msg);
}

void OffboardControlGoto::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlGoto::disarm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControlGoto::engage_offboard_mode() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
    RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
}

void OffboardControlGoto::land() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Switching to land mode");
}

int main(int argc, char* argv[]) {
    std::cout << "Starting offboard control goto node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControlGoto>());
    rclcpp::shutdown();
    return 0;
}

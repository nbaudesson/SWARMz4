# Minimum CMake version and project name
cmake_minimum_required(VERSION 3.8)
project(offboard_control_cpp)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required ROS 2 dependencies
find_package(ament_cmake REQUIRED)        # Core build system
find_package(ament_index_cpp REQUIRED)    # Package indexing
find_package(builtin_interfaces REQUIRED) # ROS message types
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)            # Matrix operations
find_package(px4_msgs REQUIRED)          # PX4 message types
find_package(rclcpp REQUIRED)            # ROS C++ client library
find_package(geometry_msgs REQUIRED)      # Standard geometry messages
find_package(yaml-cpp REQUIRED)           # YAML file parsing

# Make headers available
include_directories(include)

# Base example: Simple offboard control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs)
target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(offboard_control PUBLIC c_std_99 cxx_std_17)

# Classic hover example
add_executable(offboard_control_classic 
  src/offboard_control_classic.cpp)
ament_target_dependencies(offboard_control_classic 
  rclcpp px4_msgs)

# Frame-specific controllers
add_executable(offboard_control_frd 
  src/offboard_control_frd.cpp)
ament_target_dependencies(offboard_control_frd 
  rclcpp px4_msgs geometry_msgs)

add_executable(offboard_control_ned 
  src/offboard_control_ned.cpp)
ament_target_dependencies(offboard_control_ned 
  rclcpp px4_msgs geometry_msgs)

# Advanced waypoint controller
add_executable(offboard_control_goto 
  src/offboard_control_goto.cpp)
ament_target_dependencies(offboard_control_goto 
  rclcpp 
  px4_msgs 
  geometry_msgs 
  yaml-cpp
  ament_index_cpp)
target_link_libraries(offboard_control_goto
  yaml-cpp)

# Install executables to lib directory
install(TARGETS
  offboard_control
  offboard_control_classic
  offboard_control_frd
  offboard_control_ned
  offboard_control_goto
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (headers, missions, etc.)
install(
  DIRECTORY
    include/
    missions/
  DESTINATION share/${PROJECT_NAME}
)

# Install mission files separately to ensure they're found
install(
  DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Optional: Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skip copyright checks
  set(ament_cmake_cpplint_FOUND TRUE)    # Skip style checks
  ament_lint_auto_find_test_dependencies()
endif()

# Required: Generate package configuration
ament_package()

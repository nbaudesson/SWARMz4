cmake_minimum_required(VERSION 3.8)
project(swarmz_interfaces)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Message files
set(msg_files
  # life_point.msg => std_msgs/Int16
  # communication.msg => std_msgs/String
  "msg/Detection.msg"
  "msg/Detections.msg"
)

# Service files
set(srv_files
  "srv/Kamikaze.srv"
  "srv/Missile.srv"
  "srv/UpdateHealth.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Mark package as an ament package
ament_package()

cmake_minimum_required(VERSION 3.8)
project(bench_robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Define the executable
add_executable(arm_controller src/arm_controller.cpp)

# 3. Use ament_target_dependencies for EVERYTHING
# This replaces the need for target_link_libraries for ROS targets
ament_target_dependencies(arm_controller
  rclcpp
  moveit_ros_planning_interface
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

# 4. Install the executable
install(TARGETS arm_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
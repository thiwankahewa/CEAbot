cmake_minimum_required(VERSION 3.8)
project(arm_controlling)

# Let editors obtain the ROS and Kortex include paths used for this target.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(kortex_api REQUIRED)
find_package(kortex_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(kinova_power_logger_node src/kinova_power_logger_node.cpp)
target_compile_features(kinova_power_logger_node PUBLIC cxx_std_17)
target_compile_definitions(kinova_power_logger_node PRIVATE _OS_UNIX)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Kortex's generated headers use anonymous structs and stub parameters.
  # Keep useful warnings for this node without emitting third-party warnings.
  target_compile_options(
    kinova_power_logger_node PRIVATE
    -Wall
    -Wextra
    -Wno-pedantic
    -Wno-unused-parameter
  )
endif()
ament_target_dependencies(
  kinova_power_logger_node
  kortex_api
  kortex_driver
  rclcpp
  std_msgs
)

install(TARGETS kinova_power_logger_node DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/arm_manager
    scripts/joint_rotation_monitor
    scripts/plant_view_scanner
    scripts/remote_orbbec_capture
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

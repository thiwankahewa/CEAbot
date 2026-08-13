cmake_minimum_required(VERSION 3.8)
project(arm_controlling)

# Let editors obtain the ROS and Kortex include paths used for this target.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

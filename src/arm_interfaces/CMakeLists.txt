cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "srv/PlanToPose.srv"
  "srv/ExecutePlannedTrajectory.srv"
  "msg/PlantTarget.msg"
  "msg/PlantTargetArray.msg"
  "srv/CaptureView.srv"
  DEPENDENCIES moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

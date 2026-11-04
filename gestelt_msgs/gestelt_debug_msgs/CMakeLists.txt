cmake_minimum_required(VERSION 3.8)
project(gestelt_debug_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gestelt_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/SFCSegment.msg
  msg/SFCTrajectory.msg
  msg/BackEndTrajectoryDebug.msg
  DEPENDENCIES std_msgs geometry_msgs gestelt_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

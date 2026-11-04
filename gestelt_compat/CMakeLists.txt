cmake_minimum_required(VERSION 3.8)
project(gestelt_compat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(gestelt_msgs REQUIRED)
find_package(gestelt_debug_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_link_libraries(
  ${PROJECT_NAME}
  INTERFACE
    rclcpp::rclcpp
    rclcpp_components::component
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${trajectory_msgs_TARGETS}
    ${tf2_msgs_TARGETS}
    ${mavros_msgs_TARGETS}
    ${gestelt_msgs_TARGETS}
    ${gestelt_debug_msgs_TARGETS}
    ${decomp_ros_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)
ament_export_targets(${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
  tf2_msgs
  mavros_msgs
  gestelt_msgs
  gestelt_debug_msgs
  decomp_ros_msgs
)
ament_export_include_directories(include)
ament_package()

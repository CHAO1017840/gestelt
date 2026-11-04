cmake_minimum_required(VERSION 3.8)
project(traj_utils)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/DataDisp.msg
  msg/PolyTraj.msg
  msg/MINCOTraj.msg
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

add_library(${PROJECT_NAME}_lib SHARED src/planning_visualization.cpp)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES EXPORT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  gestelt_compat::gestelt_compat
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  Eigen3::Eigen
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(
  rclcpp std_msgs geometry_msgs visualization_msgs builtin_interfaces
  rosidl_default_runtime gestelt_compat
)
ament_package()

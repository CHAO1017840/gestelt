cmake_minimum_required(VERSION 3.8)
project(trajectory_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(gestelt_msgs REQUIRED)

add_executable(traj_server_node src/traj_server_node.cpp src/traj_server.cpp)
target_include_directories(traj_server_node PRIVATE include)
target_compile_features(traj_server_node PRIVATE cxx_std_17)
target_link_libraries(traj_server_node PRIVATE
  gestelt_compat::gestelt_compat
  gestelt_msgs::gestelt_msgs
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  Eigen3::Eigen
)

install(TARGETS traj_server_node DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rclcpp mavros_msgs Eigen3 gestelt_compat gestelt_msgs)
ament_package()

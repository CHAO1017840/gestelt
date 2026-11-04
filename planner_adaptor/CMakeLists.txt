cmake_minimum_required(VERSION 3.8)
project(planner_adaptor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(gestelt_msgs REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)

add_executable(ego_planner_adaptor_node src/ego_planner_adaptor_node.cpp)
target_include_directories(ego_planner_adaptor_node PRIVATE include)
target_compile_features(ego_planner_adaptor_node PRIVATE cxx_std_17)
target_link_libraries(ego_planner_adaptor_node PRIVATE
  gestelt_compat::gestelt_compat
  gestelt_msgs::gestelt_msgs
  traj_opt::traj_opt
  traj_utils::traj_utils
  rclcpp::rclcpp
  Eigen3::Eigen
)

install(TARGETS ego_planner_adaptor_node DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rclcpp Eigen3 gestelt_compat gestelt_msgs traj_opt traj_utils)
ament_package()

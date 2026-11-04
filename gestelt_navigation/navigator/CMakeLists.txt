cmake_minimum_required(VERSION 3.8)
project(navigator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(gestelt_msgs REQUIRED)
find_package(gestelt_debug_msgs REQUIRED)
find_package(global_planner REQUIRED)
find_package(sfc_generation REQUIRED)
find_package(traj_utils REQUIRED)
find_package(traj_opt REQUIRED)
find_package(ego_planner_fsm REQUIRED)
find_package(decomp_util REQUIRED)

add_executable(${PROJECT_NAME}_node src/navigator_node.cpp src/navigator.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
  gestelt_compat::gestelt_compat
  gestelt_msgs::gestelt_msgs
  gestelt_debug_msgs::gestelt_debug_msgs
  global_planner::global_planner
  sfc_generation::sfc_generation
  traj_utils::traj_utils
  traj_opt::traj_opt
  ego_planner_fsm::ego_planner_fsm
  decomp_util::decomp_util
  rclcpp::rclcpp
  Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp Eigen3 gestelt_compat gestelt_msgs gestelt_debug_msgs
  global_planner sfc_generation traj_utils traj_opt ego_planner_fsm decomp_util
)
ament_package()

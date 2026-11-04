cmake_minimum_required(VERSION 3.8)
project(ego_planner_fsm)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(gestelt_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(path_searching REQUIRED)
find_package(traj_opt REQUIRED)
find_package(traj_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ego_planner_manager.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gestelt_compat::gestelt_compat
  gestelt_msgs::gestelt_msgs
  grid_map::grid_map
  path_searching::path_searching
  traj_opt::traj_opt
  traj_utils::traj_utils
  tf2_ros::tf2_ros
  rclcpp::rclcpp
  Eigen3::Eigen
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(
  rclcpp tf2_ros Eigen3 gestelt_compat gestelt_msgs grid_map path_searching traj_opt traj_utils
)
ament_package()

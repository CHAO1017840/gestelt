cmake_minimum_required(VERSION 3.8)
project(grid_map)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(bonxai REQUIRED)
find_package(ikd_tree REQUIRED)
find_package(logger REQUIRED)

include_directories(${pcl_conversions_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/grid_map.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pcl_conversions_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gestelt_compat::gestelt_compat
  bonxai::bonxai
  ikd_tree::ikd_tree
  logger::logger
  tf2_ros::tf2_ros
  message_filters::message_filters
  ${pcl_conversions_LIBRARIES}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
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
  rclcpp tf2_ros message_filters sensor_msgs nav_msgs std_msgs pcl_conversions
  Eigen3 gestelt_compat bonxai ikd_tree logger
)
ament_package()

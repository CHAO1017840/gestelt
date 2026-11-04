cmake_minimum_required(VERSION 3.8)
project(fake_map)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(gestelt_compat REQUIRED)

include_directories(${pcl_conversions_INCLUDE_DIRS})

add_library(fake_laser SHARED src/fake_laser.cpp)
target_include_directories(fake_laser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(fake_laser PUBLIC cxx_std_17)
target_link_libraries(fake_laser PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(fake_sensor_node src/fake_sensor_node.cpp src/fake_sensor.cpp)
target_include_directories(fake_sensor_node PRIVATE include)
target_compile_features(fake_sensor_node PRIVATE cxx_std_17)
target_link_libraries(fake_sensor_node PRIVATE
  gestelt_compat::gestelt_compat
  fake_laser
  tf2_ros::tf2_ros
  ${pcl_conversions_LIBRARIES}
  rclcpp::rclcpp
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(fake_map_publisher src/fake_map_publisher.cpp)
target_compile_features(fake_map_publisher PRIVATE cxx_std_17)
target_link_libraries(fake_map_publisher PRIVATE
  gestelt_compat::gestelt_compat
  ${pcl_conversions_LIBRARIES}
  rclcpp::rclcpp
  ${PCL_LIBRARIES}
)

add_executable(fake_map_generator src/fake_map_generator.cpp)
target_compile_features(fake_map_generator PRIVATE cxx_std_17)
target_link_libraries(fake_map_generator PRIVATE
  gestelt_compat::gestelt_compat
  rclcpp::rclcpp
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(TARGETS fake_laser
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS fake_sensor_node fake_map_publisher fake_map_generator
  DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(rclcpp tf2_ros pcl_conversions Eigen3 gestelt_compat)
ament_package()

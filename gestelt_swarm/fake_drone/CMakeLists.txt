cmake_minimum_required(VERSION 3.8)
project(fake_drone)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)

add_executable(fake_drone_node src/fake_drone_node.cpp src/fake_drone.cpp)
target_include_directories(fake_drone_node PRIVATE include)
target_compile_features(fake_drone_node PRIVATE cxx_std_17)
target_link_libraries(fake_drone_node PRIVATE
  gestelt_compat::gestelt_compat
  tf2_ros::tf2_ros
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  Eigen3::Eigen
)

install(TARGETS fake_drone_node DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rclcpp tf2_ros mavros_msgs Eigen3 gestelt_compat)
ament_package()

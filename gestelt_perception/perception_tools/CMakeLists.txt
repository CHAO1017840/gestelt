cmake_minimum_required(VERSION 3.8)
project(perception_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(gestelt_compat REQUIRED)

include_directories(${pcl_conversions_INCLUDE_DIRS})

add_executable(cloud_downsampler src/cloud_downsampler.cpp)
target_compile_features(cloud_downsampler PRIVATE cxx_std_17)
target_link_libraries(cloud_downsampler PRIVATE
  gestelt_compat::gestelt_compat
  ${pcl_conversions_LIBRARIES}
  rclcpp::rclcpp
  ${PCL_LIBRARIES}
)

install(TARGETS cloud_downsampler DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rclcpp pcl_conversions gestelt_compat)
ament_package()

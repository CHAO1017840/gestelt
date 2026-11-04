cmake_minimum_required(VERSION 3.8)
project(radxa_utils)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gestelt_compat REQUIRED)

add_executable(listener src/listener.cpp)
add_executable(talker src/talker.cpp)
target_link_libraries(listener PRIVATE gestelt_compat::gestelt_compat rclcpp::rclcpp)
target_link_libraries(talker PRIVATE gestelt_compat::gestelt_compat rclcpp::rclcpp)

install(TARGETS listener talker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rclcpp gestelt_compat)
ament_package()

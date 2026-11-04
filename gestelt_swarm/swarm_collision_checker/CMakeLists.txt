cmake_minimum_required(VERSION 3.8)
project(swarm_collision_checker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/swarm_collision_checker.cpp
  src/swarm_collision_checker_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
  gestelt_compat::gestelt_compat
  rclcpp::rclcpp
  Eigen3::Eigen
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rclcpp Eigen3 gestelt_compat)
ament_package()

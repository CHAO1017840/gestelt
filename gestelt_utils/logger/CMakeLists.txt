cmake_minimum_required(VERSION 3.8)
project(logger)

find_package(ament_cmake REQUIRED)
find_package(gestelt_compat REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE gestelt_compat::gestelt_compat)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets NAMESPACE ${PROJECT_NAME}::)
ament_export_dependencies(gestelt_compat rclcpp)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(global_planner)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gestelt_compat REQUIRED)
find_package(grid_map REQUIRED)
find_package(jps3d REQUIRED)
find_package(logger REQUIRED)

add_library(${PROJECT_NAME} SHARED src/a_star.cpp src/jps_wrapper.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC
  gestelt_compat::gestelt_compat
  grid_map::grid_map
  jps3d::jps3d
  logger::logger
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
ament_export_dependencies(Eigen3 gestelt_compat grid_map jps3d logger)
ament_package()

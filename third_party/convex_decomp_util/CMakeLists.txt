cmake_minimum_required(VERSION 3.8)
project(convex_decomp_util)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)

add_library(${PROJECT_NAME} SHARED src/convex_decomp.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen decomp_util::decomp_util)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 decomp_util)
ament_package()

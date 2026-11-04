cmake_minimum_required(VERSION 3.8)
project(trajectory_inspector)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/trajectory_inspector.py DESTINATION lib/${PROJECT_NAME})
ament_package()

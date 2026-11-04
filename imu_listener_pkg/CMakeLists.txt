cmake_minimum_required(VERSION 3.8)
project(imu_listener_pkg)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)

install(PROGRAMS
  src/ai_imu_client.py
  src/ai_imu_velocity.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  src/realtime_imu_inference_buffer.py
  src/strapdown_velocity_estimator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(inference test/test_inference.py TIMEOUT 60)
  ament_add_pytest_test(
    velocity_estimator
    test/test_velocity_estimator.py
    TIMEOUT 60
  )
endif()

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclpy
  sensor_msgs
  std_msgs
  ai_msgs
)
ament_package()

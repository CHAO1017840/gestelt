cmake_minimum_required(VERSION 3.8)
project(gestelt_bringup)

find_package(ament_cmake REQUIRED)

file(GLOB GESTELT_PYTHON_SCRIPTS CONFIGURE_DEPENDS src/*.py)
file(GLOB GESTELT_SCENARIO_SCRIPTS CONFIGURE_DEPENDS src/scenarios/*.py)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY simulation DESTINATION share/${PROJECT_NAME})
install(DIRECTORY flight_params DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/base_station.sh scripts/debug_plan.sh scripts/offboard.sh
  scripts/scenario.sh scripts/scenario_hitl_base.sh scripts/scenario_hitl_drone.sh
  scripts/scenario_sitl_drone.sh scripts/sitl_single_drone.sh
  scripts/install_raynor_sitl.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS ${GESTELT_PYTHON_SCRIPTS} DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS ${GESTELT_SCENARIO_SCRIPTS} DESTINATION share/${PROJECT_NAME}/scenarios)
ament_package()

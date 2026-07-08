cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch simulation config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  simulation/scripts/gz_pose_to_gps.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_pose_to_gps
)

install(PROGRAMS
  simulation/scripts/set_gazebo_simulation_speed.py
  simulation/scripts/wait_for_fsm_fault_shutdown.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

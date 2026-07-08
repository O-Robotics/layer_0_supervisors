cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds missions
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gz_pose_to_gps.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_pose_to_gps
)

install(PROGRAMS
  scripts/set_gazebo_simulation_speed.py
  scripts/wait_for_fsm_fault_shutdown.py
  scripts/world_pose_to_sim_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


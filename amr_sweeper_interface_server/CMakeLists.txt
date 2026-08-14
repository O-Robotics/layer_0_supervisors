cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_interface_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/backend_node.py
  scripts/frontend_http_node.py
  scripts/mqtt_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY assets launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY assets/
  DESTINATION lib/${PROJECT_NAME}/assets
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

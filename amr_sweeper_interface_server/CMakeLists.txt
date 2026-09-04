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
  include(CTest)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_gaussian_splat_artifact_roots
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gaussian_splat_artifact_roots.py
  )
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg")
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

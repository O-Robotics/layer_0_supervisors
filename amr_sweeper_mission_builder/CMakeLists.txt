cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_mission_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

add_executable(mission_builder_node
  src/mission_builder_node.cpp
)
target_include_directories(mission_builder_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)
target_link_directories(mission_builder_node PUBLIC ${GEOGRAPHICLIB_LIBRARY_DIRS})
target_link_libraries(mission_builder_node ${GEOGRAPHICLIB_LIBRARIES})
ament_target_dependencies(mission_builder_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS
  mission_builder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs std_srvs)
ament_package()

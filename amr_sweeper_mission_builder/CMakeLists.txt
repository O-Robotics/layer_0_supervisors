cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_mission_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(amr_sweeper_fsm REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BuildGaussianSplat.srv"
  "srv/PauseGaussianSplatBuild.srv"
  "srv/ResumeGaussianSplatBuild.srv"
)

pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)
pkg_check_modules(LIBZIP REQUIRED libzip)

add_executable(vda5050_parser_node
  src/vda5050_parser_node.cpp
)
target_include_directories(vda5050_parser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
  ${LIBZIP_INCLUDE_DIRS}
)
target_link_directories(vda5050_parser_node PUBLIC
  ${GEOGRAPHICLIB_LIBRARY_DIRS}
  ${LIBZIP_LIBRARY_DIRS}
)
target_link_libraries(vda5050_parser_node
  ${GEOGRAPHICLIB_LIBRARIES}
  ${LIBZIP_LIBRARIES}
)
ament_target_dependencies(vda5050_parser_node
  rclcpp
  std_msgs
  std_srvs
)

install(TARGETS
  vda5050_parser_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gaussian_splat_builder_node
  scripts/gaussian_splat_worker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  include(CTest)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_gaussian_splat_builder_node
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gaussian_splat_builder_node.py
  )
  add_executable(test_mission_package_reader
    test/test_mission_package_reader.cpp
  )
  target_include_directories(test_mission_package_reader PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBZIP_INCLUDE_DIRS}
  )
  target_link_directories(test_mission_package_reader PRIVATE ${LIBZIP_LIBRARY_DIRS})
  target_link_libraries(test_mission_package_reader ${LIBZIP_LIBRARIES})
  add_test(NAME test_mission_package_reader COMMAND test_mission_package_reader)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/setup.cfg")
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  amr_sweeper_fsm
  rclcpp
  rosidl_default_runtime
  std_msgs
  std_srvs
)
ament_package()

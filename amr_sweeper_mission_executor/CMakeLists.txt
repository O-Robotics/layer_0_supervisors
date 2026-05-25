cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_mission_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(amr_sweeper_fsm REQUIRED)
find_package(amr_sweeper_safety_msgs REQUIRED)
find_package(amr_sweeper_vda5050_parser REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EndMission.srv"
  "srv/ExecuteMission.srv"
  "srv/ListExecutableMissions.srv"
  "srv/ListManualMissions.srv"
  "srv/PrepareManualMission.srv"
  "srv/UploadVda5050Mission.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(mission_executor_node
  src/mission_executor_node.cpp
)
target_include_directories(mission_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mission_executor_node
  ament_index_cpp
  amr_sweeper_fsm
  amr_sweeper_safety_msgs
  amr_sweeper_vda5050_parser
  geometry_msgs
  nav_msgs
  rclcpp
  rcl_interfaces
  std_srvs
)
target_link_libraries(mission_executor_node
  ${cpp_typesupport_target}
)

add_dependencies(mission_executor_node ${PROJECT_NAME})

install(TARGETS
  mission_executor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mission_web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

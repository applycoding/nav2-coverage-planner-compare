# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(ncp_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanGlobal.srv"
  "srv/PlanCoverage.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

find_path(NCP_INCLUDE ncp/types.hpp
  PATHS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES include
  REQUIRED)
find_library(NCP_CORE_LIB ncp_core
  PATHS ${CMAKE_PREFIX_PATH}
  PATH_SUFFIXES lib
  REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(ncp_planner_node src/planner_node.cpp)
target_include_directories(ncp_planner_node PRIVATE ${NCP_INCLUDE})
target_link_libraries(ncp_planner_node ${NCP_CORE_LIB} yaml-cpp)
ament_target_dependencies(ncp_planner_node rclcpp nav_msgs geometry_msgs visualization_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ncp_planner_node ${cpp_typesupport_target})

install(TARGETS ncp_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/smoke_plan_global.py DESTINATION lib/${PROJECT_NAME})

ament_package()

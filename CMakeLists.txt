# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.22)
project(nav2_coverage_planner_compare VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(NCP_BUILD_PYTHON "Build pybind11 module" ON)
option(NCP_BUILD_TESTS "Build GoogleTest suite" ON)
option(NCP_BUILD_ROS "Build ROS 2 wrapper" OFF)

include(FetchContent)
set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)

set(GTEST_COMMIT 063de7e9578f82b369302001269680b4b1553359)
set(PYBIND11_COMMIT 97bf890db679505a14dfe547a5e77bb2bd05dc90)
set(JSON_COMMIT 55f93686c01528224f448c19128836e7df245f72)
set(YAMLCPP_COMMIT 56e3bb550c91fd7005566f19c079cb7a503223cf)

if(NCP_BUILD_TESTS)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG ${GTEST_COMMIT}
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG ${JSON_COMMIT}
)
FetchContent_MakeAvailable(nlohmann_json)

FetchContent_Declare(yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG ${YAMLCPP_COMMIT}
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

add_library(ncp_core STATIC
  # PIC required for pybind11 shared module
  cpp/src/types.cpp
  cpp/src/grid_map.cpp
  cpp/src/costmap.cpp
  cpp/src/collision.cpp
  cpp/src/metrics.cpp
  cpp/src/validators.cpp
  cpp/src/rng.cpp
  cpp/src/planners/graph_search.cpp
  cpp/src/planners/lazy_theta_star.cpp
  cpp/src/planners/rrt_star.cpp
  cpp/src/planners/global_planner.cpp
  cpp/src/coverage/coverage_planner.cpp
)

set_target_properties(ncp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(ncp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ncp_core PUBLIC yaml-cpp)
target_link_libraries(ncp_core PRIVATE nlohmann_json::nlohmann_json)

add_executable(ncp_dump_fixture cpp/tools/ncp_dump_fixture.cpp)
target_link_libraries(ncp_dump_fixture PRIVATE ncp_core nlohmann_json::nlohmann_json)

if(NCP_BUILD_TESTS)
  enable_testing()
  add_subdirectory(cpp/test)
endif()

if(NCP_BUILD_PYTHON)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG ${PYBIND11_COMMIT}
  )
  FetchContent_MakeAvailable(pybind11)
  add_subdirectory(python/bindings)
endif()

install(DIRECTORY cpp/include/ncp DESTINATION include)

install(
  TARGETS ncp_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(FILES cmake/ncpConfig.cmake.in DESTINATION lib/cmake/ncp RENAME ncpConfig.cmake)

if(NCP_BUILD_ROS)
  add_subdirectory(ros2/ncp_planner)
endif()

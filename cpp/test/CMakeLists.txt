# SPDX-License-Identifier: Apache-2.0
include(GoogleTest)

add_executable(ncp_tests
  test_grid_costmap.cpp
  test_planners.cpp
  test_coverage.cpp
  test_rng.cpp
  test_criteria.cpp
)

target_compile_definitions(ncp_tests PRIVATE NCP_FIXTURES_DIR="${CMAKE_SOURCE_DIR}/fixtures/maps")
target_link_libraries(ncp_tests PRIVATE ncp_core GTest::gtest_main)

gtest_discover_tests(ncp_tests PROPERTIES TIMEOUT 2)

# SPDX-License-Identifier: Apache-2.0
pybind11_add_module(ncp_py bindings.cpp)

target_link_libraries(ncp_py PRIVATE ncp_core)

set_target_properties(ncp_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
  OUTPUT_NAME "ncp_core")

add_custom_command(TARGET ncp_py POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_SOURCE_DIR}/python/ncp"
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ncp_py>
          "${CMAKE_SOURCE_DIR}/python/ncp/ncp_core${CMAKE_SHARED_LIBRARY_SUFFIX}")

# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20)
project(nrf54l15_support VERSION 0.1.0 LANGUAGES C)
add_subdirectory(lib/nrf54l15)
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests/host)
endif()

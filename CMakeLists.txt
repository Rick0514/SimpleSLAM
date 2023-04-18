cmake_minimum_required(VERSION 3.4.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER clang-12)
set(CMAKE_CXX_COMPILER clang++-12)

project(NaiveSLAM)

message(STATUS "compiler is " ${CMAKE_CXX_COMPILER_ID})
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(COMMON_HEADER ${ROOT_DIR}/common)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(PCR)
add_subdirectory(filter)
add_subdirectory(dataproxy)
add_subdirectory(frontend)
# add_subdirectory(backend)
add_subdirectory(test)

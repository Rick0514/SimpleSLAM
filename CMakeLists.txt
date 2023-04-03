cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER clang-12)
set(CMAKE_CXX_COMPILER clang++-12)


project(PCR)

message(STATUS "compiler is " ${CMAKE_CXX_COMPILER_ID})

set(COMMON_HEADER ${PCR_SOURCE_DIR}/common)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(PCR)
add_subdirectory(filter)
add_subdirectory(dataproxy)
add_subdirectory(test)
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(third_parties/pclomp)

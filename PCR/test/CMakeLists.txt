cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER clang-12)
set(CMAKE_CXX_COMPILER clang++-12)

project(pcr_test)

set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

add_executable(test test.cpp)

target_include_directories(test PUBLIC
    ../../common
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(test PUBLIC
    ${PCL_LIBRARIES}
    spdlog
)




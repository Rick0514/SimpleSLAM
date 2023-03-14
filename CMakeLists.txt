cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 14)

project(PCR)

find_package(Eigen3 REQUIRED)

add_subdirectory(PCR)
add_subdirectory(test)
add_subdirectory(third_parties/pclomp)





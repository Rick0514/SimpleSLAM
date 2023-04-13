cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER clang-12)
set(CMAKE_CXX_COMPILER clang++-12)

project(benchmark)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

find_package(PCL REQUIRED COMPONENTS common io kdtree filters)

find_package(benchmark REQUIRED)

link_libraries(
    pthread
    benchmark::benchmark
    benchmark::benchmark_main
)

add_executable(rospub rospub.cpp)
target_include_directories(rospub PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(rospub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(kdtree kdtree.cpp)
target_include_directories(kdtree PUBLIC
    ${PCL_INCLUDE_DIRS}
    ../../common
    ../../third_parties/nanoflann/include
)
target_link_libraries(kdtree
    ${PCL_LIBRARIES}
)
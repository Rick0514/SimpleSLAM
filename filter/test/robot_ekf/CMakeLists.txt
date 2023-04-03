cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER clang-12)
set(CMAKE_CXX_COMPILER clang++-12)

project(robot_ekf)

set(CMAKE_BUILD_TYPE Debug)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

add_definitions(-DDATA_FILE="${CMAKE_CURRENT_SOURCE_DIR}/../data")

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    tf
    nav_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/odom_estimation.cpp src/odom_estimation_node.cpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    spdlog
)

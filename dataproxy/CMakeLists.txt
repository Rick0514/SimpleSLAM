project(dataproxy)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    sensor_msgs
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(dataproxy SHARED ${srcs})

message(STATUS "filter: " ${filter_INCLUDE_DIRS})

target_include_directories(dataproxy PUBLIC
    ${COMMON_HEADER}
    ${filter_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(dataproxy
    ${catkin_LIBRARIES}   
    ${PCL_LIBRARIES}
)




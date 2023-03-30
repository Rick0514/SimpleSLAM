project(dataproxy)

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    sensor_msgs
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(dataproxy STATIC ${srcs})

target_include_directories(dataproxy PUBLIC
    ${COMMON_HEADER}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(dataproxy
    ${catkin_LIBRARIES}   
    ${PCL_LIBRARIES}
)




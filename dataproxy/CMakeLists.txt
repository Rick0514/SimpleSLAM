project(dataproxy)

find_package(PCL REQUIRED COMPONENTS common filters)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    sensor_msgs
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

message(STATUS "filter: " ${filter_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${COMMON_HEADER}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${config_INCLUDE_DIRS}
    ${filter_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    eskf
)
    
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${catkin_LIBRARIES}
)





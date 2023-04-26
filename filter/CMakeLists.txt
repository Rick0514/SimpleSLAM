project(filter)

set(KalmanFilterHeaders ${ROOT_DIR}/third_parties/KalmanFilter/include)

# for this header-only library
set(filter_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${KalmanFilterHeaders}
    PARENT_SCOPE
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    sensor_msgs
    nav_msgs
)

add_executable(ekf test/ekf.cpp)

target_compile_definitions(ekf PRIVATE DATA_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data")

target_include_directories(ekf PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${COMMON_HEADER}
    ${KalmanFilterHeaders}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ekf PUBLIC
    spdlog
    ${catkin_LIBRARIES}
)



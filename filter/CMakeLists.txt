project(filter)

set(KalmanFilterHeaders ../third_parties/KalmanFilter/include)

# target_compile_definitions(pcr PRIVATE LOG_LEVEL=1)
# set(CMAKE_BUILD_TYPE Debug)

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


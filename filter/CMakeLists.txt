project(filter)

set(KalmanFilterHeaders ../third_parties/KalmanFilter/include)

# target_compile_definitions(pcr PRIVATE LOG_LEVEL=1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
)

add_executable(ekf test/ekf.cpp)

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


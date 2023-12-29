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

add_library(eskf SHARED src/Eskf.cpp)
target_include_directories(eskf PRIVATE
    include
    ${COMMON_HEADER}
    ${EIGEN3_INCLUDE_DIRS}
    ${config_INCLUDE_DIRS}
)

target_include_directories(eskf PUBLIC
    ${Sophus}
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

add_executable(test_eskf test/test_eskf.cpp)

target_include_directories(test_eskf PRIVATE
    include
    ${COMMON_HEADER}
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus}
)
target_link_libraries(test_eskf PUBLIC
    eskf
)
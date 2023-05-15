cmake_minimum_required(VERSION 3.0.2)

project(app)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    rosbag
)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${ROOT_DIR}/dataproxy/include
    ${ROOT_DIR}/frontend/include
    ${ROOT_DIR}/backend/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    dataproxy
    frontend
    backend
    loopclosure
    ${catkin_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE LOG_FILE="${ROOT_DIR}/test/data/slam.log")
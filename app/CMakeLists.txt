cmake_minimum_required(VERSION 3.0.2)

project(app)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    rosbag
)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${ROOT_DIR}/frontend/include
    ${ROOT_DIR}/backend/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    frontend
    backend
    ${catkin_LIBRARIES}
)
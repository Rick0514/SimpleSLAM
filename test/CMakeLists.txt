cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 14)

project(test)

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
# )

find_package(PCL REQUIRED COMPONENTS common io filters visualization)

add_definitions(-DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(align align.cpp)
target_include_directories(align PUBLIC
    ${PCR_SOURCE_DIR}/PCR/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(align PUBLIC
    pcr
    spdlog::spdlog
    ${PCL_LIBRARIES}
)

add_executable(eigen eigen.cpp)
target_include_directories(eigen PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${COMMON_HEADER}
)

add_executable(test_dataproxy dataproxy.cpp)
target_include_directories(test_dataproxy PUBLIC
    # ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${COMMON_HEADER}
    ${PCR_SOURCE_DIR}/dataproxy/include
)

message(STATUS "link dir: " ${CMAKE_BINARY_DIR}/dataproxy)
target_link_directories(test_dataproxy PUBLIC
    ${CMAKE_BINARY_DIR}/dataproxy
)
get_target_property(result test_dataproxy LINK_DIRECTORIES)
message(STATUS "result: ${result}")

target_link_libraries(test_dataproxy
    ${catkin_LIBRARIES}
    dataproxy
)


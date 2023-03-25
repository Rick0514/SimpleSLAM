cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE Debug)

project(test)

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
    ${EIGEN3_INCLUDE_DIRS}
    ${COMMON_HEADER}
    ${PCR_SOURCE_DIR}/dataproxy/include
)
target_link_libraries(test_dataproxy
    ${catkin_LIBRARIES}
    dataproxy
)

add_executable(safedeque safedeque.cpp)
target_include_directories(safedeque PUBLIC
    ${COMMON_HEADER}
)
target_link_libraries(safedeque
    pthread
)

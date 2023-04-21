cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(comp)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters visualization)

add_executable(liorf_scan2map liorf_scan2map.cpp)
target_include_directories(liorf_scan2map PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(liorf_scan2map PUBLIC
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    OpenMP::OpenMP_CXX
)

target_compile_definitions(liorf_scan2map PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data/")

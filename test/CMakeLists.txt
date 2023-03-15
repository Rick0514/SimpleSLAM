cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 14)

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

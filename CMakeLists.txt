cmake_minimum_required(VERSION 3.4.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(SimpleSLAM)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")
add_compile_definitions(NDEBUG)

message(STATUS "compiler is " ${CMAKE_CXX_COMPILER_ID})

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    message(STATUS "use ccache to speed up re-compilation!")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(COMMON_HEADER ${ROOT_DIR}/common)

option(USE_BAG "USE ROSBAG OR NOT" OFF)
message("USE ROSBAG OR NOT: " ${USE_BAG})

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
set(NANOFLANN ${CMAKE_CURRENT_SOURCE_DIR}/third_parties/nanoflann/include)
set(Sophus ${CMAKE_CURRENT_SOURCE_DIR}/third_parties/sophus/include)

add_definitions(-DCONFIG_FILE="${ROOT_DIR}/config/params.json")

if(USE_BAG)
	add_definitions(-DUSE_BAG)
endif()

add_subdirectory(config)
add_subdirectory(PCR)
add_subdirectory(filter)
add_subdirectory(dataproxy)
add_subdirectory(frontend)
# add_subdirectory(backend)
add_subdirectory(test)
add_subdirectory(app)

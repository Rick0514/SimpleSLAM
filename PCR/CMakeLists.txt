
find_package(PCL REQUIRED COMPONENTS common kdtree filters)

set(COMMON_HEADER ${PCR_SOURCE_DIR}/common)

set(CMAKE_BUILD_TYPE Release)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.hpp)
add_library(pcr SHARED ${srcs})

target_compile_definitions(pcr PRIVATE LOG_LEVEL=1)

target_link_libraries(pcr PUBLIC
    ${PCL_LIBRARIES}
    spdlog::spdlog
    pclomp::registration
)

target_include_directories(pcr PUBLIC
    include
    ${COMMON_HEADER}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)



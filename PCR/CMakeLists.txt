project(PCR)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_subdirectory(../third_parties/pclomp pclomp)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

# target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_DIR="${ROOT_DIR}/test/data")

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    spdlog
    pclomp::registration
)
target_link_libraries(${PROJECT_NAME} PRIVATE 
    OpenMP::OpenMP_CXX
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ../third_parties/nanoflann/include
)



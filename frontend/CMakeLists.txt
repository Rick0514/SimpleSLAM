project(frontend)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED COMPONENTS common io filters)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    dataproxy
    PCR
    ${PCL_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_PC="${ROOT_DIR}/test/data")
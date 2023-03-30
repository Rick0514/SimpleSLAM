project(backend)

find_package(PCL REQUIRED COMPONENTS common kdtree filters io)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} STATIC ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
)
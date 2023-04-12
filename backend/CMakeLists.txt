project(backend)

find_package(PCL REQUIRED COMPONENTS common kdtree filters io)
find_package(GTSAM REQUIRED)

set(NANOFLANN ../third_parties/nanoflann/include)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} STATIC ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${NANOFLANN}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PCL_LIBRARIES}
    gtsam
)
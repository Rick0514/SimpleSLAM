project(backend)

set(CMAKE_BUILD_TYPE Release)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(GTSAM REQUIRED)

set(NANOFLANN ../third_parties/nanoflann/include)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${PCL_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
    ${NANOFLANN}
    ${ROOT_DIR}/frontend/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    gtsam
    frontend
)
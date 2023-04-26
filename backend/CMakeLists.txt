project(backend)

# find_package(OpenMP)
find_package(GTSAM REQUIRED)

set(NANOFLANN ../third_parties/nanoflann/include)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${GTSAM_INCLUDE_DIRS}
    ${NANOFLANN}
    ${ROOT_DIR}/frontend/include
)
# target_link_libraries(${PROJECT_NAME} PRIVATE 
#     OpenMP::OpenMP_CXX
# )

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtsam
    frontend    
)
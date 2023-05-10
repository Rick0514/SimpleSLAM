project(backend)

# find_package(OpenMP)
find_package(GTSAM REQUIRED)

set(NANOFLANN ../third_parties/nanoflann/include)

add_library(${PROJECT_NAME} SHARED
    include/backend/Backend.hpp
    src/Backend.cpp
)

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
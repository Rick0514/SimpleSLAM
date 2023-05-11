project(backend)

# find_package(OpenMP)
find_package(GTSAM REQUIRED)

set(NANOFLANN ../third_parties/nanoflann/include)

add_library(${PROJECT_NAME} SHARED
    include/backend/Backend.hpp
    src/Backend.cpp
)

add_library(loopclosure SHARED
    src/LoopClosureManager.cpp
    src/ScanContext.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${GTSAM_INCLUDE_DIRS}
    ${NANOFLANN}
    ${ROOT_DIR}/frontend/include
)

target_include_directories(loopclosure PUBLIC
    include
    ${COMMON_HEADER}
    ${NANOFLANN}
    ${ROOT_DIR}/PCR/include
    ${ROOT_DIR}/frontend/include
)

target_link_libraries(loopclosure PRIVATE 
    OpenMP::OpenMP_CXX
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    gtsam
    loopclosure    
)

target_link_libraries(loopclosure PUBLIC
    frontend
)
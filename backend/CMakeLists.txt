project(backend)

find_package(OpenMP)
find_package(GTSAM REQUIRED)

add_library(${PROJECT_NAME} SHARED
    include/backend/Backend.hpp
    src/Backend.cpp
)

add_library(loopclosure SHARED
    src/LoopClosureManager.cpp
    src/ScanContext.cpp
)

target_include_directories(loopclosure PUBLIC
    include
    ${COMMON_HEADER}    
    ${config_INCLUDE_DIRS}
)

target_include_directories(loopclosure PRIVATE
    ${NANOFLANN}
    ${ROOT_DIR}/PCR/include
    ${ROOT_DIR}/frontend/include
)

target_link_libraries(loopclosure PRIVATE
    PCR
    frontend
    OpenMP::OpenMP_CXX
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${ROOT_DIR}/frontend/include
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GTSAM_INCLUDE_DIRS}
    ${NANOFLANN}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    gtsam
    frontend
    loopclosure    
)

project(PCR)

add_compile_options(-O3)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_subdirectory(../third_parties/pclomp pclomp)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} SHARED ${srcs})

# target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_DIR="${ROOT_DIR}/test/data")

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${PCL_LIBRARIES}
    spdlog
)
    
target_link_libraries(${PROJECT_NAME} PRIVATE 
    pclomp::registration
    OpenMP::OpenMP_CXX
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${COMMON_HEADER}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
    
target_include_directories(${PROJECT_NAME} PRIVATE
    ../third_parties/nanoflann/include
)



project(frontend)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/${PROJECT_NAME}/*.hpp)
add_library(${PROJECT_NAME} STATIC ${srcs})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${PCR_SOURCE_DIR}/backend/include
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    dataproxy
    backend
    pcr
)

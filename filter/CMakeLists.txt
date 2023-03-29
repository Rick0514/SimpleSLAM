project(filter)

add_subdirectory(../third_parties/KalmanFilter)

# target_compile_definitions(pcr PRIVATE LOG_LEVEL=1)

add_executable(ekf test/ekf.cpp)

target_include_directories(ekf PRIVATE
    include
    ${COMMON_HEADER}
    ${KalmanFilterBase}
    ${KalmanFilterModel}
    ${KalmanFilterEKF}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ekf PUBLIC
    spdlog
)


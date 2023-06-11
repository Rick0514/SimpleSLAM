project(test)

find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(catkin REQUIRED COMPONENTS
    rosbag
)

add_definitions(-DDATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

add_executable(align align.cpp)
target_include_directories(align PUBLIC
    ${NANOFLANN}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(align
    pthread
    PCR
    ${PCL_LIBRARIES}
)

add_executable(eigen eigen.cpp)
target_include_directories(eigen PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${COMMON_HEADER}
)

add_executable(test_dataproxy dataproxy.cpp)
target_include_directories(test_dataproxy PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${COMMON_HEADER}
    ${ROOT_DIR}/dataproxy/include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(test_dataproxy
    ${catkin_LIBRARIES}
    dataproxy
)

add_executable(safedeque safedeque.cpp)
target_include_directories(safedeque PUBLIC
    ${COMMON_HEADER}
)
target_link_libraries(safedeque
    pthread
)

add_executable(testlog logger.cpp)
target_include_directories(testlog PUBLIC
    ${COMMON_HEADER}
)
target_link_libraries(testlog
    pthread
    spdlog
)
target_compile_definitions(testlog PRIVATE SPDLOG_ACTIVE_LEVEL=0)
# target_compile_definitions(testlog PRIVATE LOG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/testlog.log")


add_executable(loc loc.cpp)
target_include_directories(loc PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(loc
    ${catkin_LIBRARIES}
    dataproxy
    frontend
)
# target_compile_definitions(loc PRIVATE LOG_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/loc.log")

add_executable(thread thread.cpp)
target_include_directories(thread PUBLIC
    ${COMMON_HEADER}
)
target_link_libraries(thread
    spdlog
    pthread
)
    
add_executable(edp edp.cpp)
target_include_directories(edp PRIVATE
    ${COMMON_HEADER}
    ${ROOT_DIR}/dataproxy/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(edp
    dataproxy
    ${catkin_LIBRARIES}
)

add_executable(config config.cpp)
target_include_directories(config PRIVATE
    ${config_INCLUDE_DIRS}
)
target_link_libraries(config spdlog)


find_package(Boost REQUIRED filesystem)

add_executable(file file.cpp)
target_include_directories(file PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${COMMON_HEADER}
    ${config_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(file
    Boost::filesystem
    spdlog
)

add_executable(lcm lcm.cpp)
target_include_directories(lcm PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${ROOT_DIR}/frontend/include
    ${ROOT_DIR}/backend/include
)
target_link_libraries(lcm
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    frontend
    loopclosure
)

add_executable(visgm vis_globalmap.cpp)
target_include_directories(visgm PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${config_INCLUDE_DIRS}
)
target_link_libraries(visgm
    ${PCL_LIBRARIES}
)
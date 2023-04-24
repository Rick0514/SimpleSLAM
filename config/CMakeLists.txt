# for this header-only library
set(config_INCLUDE_DIRS
    ${ROOT_DIR}
    ${COMMON_HEADER}
    ${ROOT_DIR}/third_parties/nlohmann/include
    PARENT_SCOPE
)

add_definitions(-DCONFIG="${CMAKE_CURRENT_SOURCE_DIR}/params.json")
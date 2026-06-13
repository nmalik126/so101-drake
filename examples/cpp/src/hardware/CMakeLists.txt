find_package(drake CONFIG REQUIRED)

add_library(hardware
    so101_lcm.cpp
)

target_include_directories(hardware
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/../../lcmdefs
)

target_link_libraries(hardware
    PUBLIC
        drake::drake
)

find_package(drake CONFIG REQUIRED)

add_library(optimization
    trajectory_optimization.cpp
)

target_include_directories(optimization
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(optimization
    PUBLIC
        drake::drake
)

find_package(drake CONFIG REQUIRED)

add_library(kinematics
    inverse_kinematics.cpp
)

target_include_directories(kinematics
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

target_link_libraries(kinematics
    PUBLIC
        drake::drake
)

find_package(drake CONFIG REQUIRED)
find_package(ompl REQUIRED)

add_library(planning
    ompl_planning.cpp
    ompl_validity_checker.cpp
)

target_include_directories(planning
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(planning
    PUBLIC
        drake::drake
        ${OMPL_LIBRARIES}
)

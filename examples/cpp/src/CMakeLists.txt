find_package(drake CONFIG REQUIRED)
find_package(ompl REQUIRED)

add_subdirectory(hardware)
add_subdirectory(kinematics)
add_subdirectory(planning)
add_subdirectory(optimization)

add_library(helpers
    scenario_helpers.cpp
    motion_planning_helpers.cpp
)
target_include_directories(helpers
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(helpers
    PRIVATE
        kinematics
        planning
        optimization
)

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer drake::drake)

add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test
    PRIVATE
        helpers
        kinematics
)

add_executable(ompl_test ompl_test.cpp)
target_link_libraries(ompl_test
    PRIVATE
        helpers
        planning
)

add_executable(trajopt_test trajopt_test.cpp)
target_link_libraries(trajopt_test
    PRIVATE
        helpers
        optimization
)

add_executable(motion_planning_test motion_planning_test.cpp)
target_link_libraries(motion_planning_test
    PRIVATE
        helpers
        kinematics
        planning
        optimization
)

add_executable(simulation_demo simulation_demo.cpp)
target_link_libraries(simulation_demo
    PRIVATE
        helpers
        kinematics
        planning
        optimization
)

add_executable(hardware_test hardware_test.cpp)
target_link_libraries(hardware_test
    PRIVATE
        helpers
        hardware
)

add_executable(hardware_demo hardware_demo.cpp)
target_link_libraries(hardware_demo
    PRIVATE
        helpers
        hardware
        kinematics
        planning
        optimization
)

find_package(drake CONFIG REQUIRED)

add_subdirectory(hardware)
add_subdirectory(kinematics)
add_subdirectory(planning)
add_subdirectory(optimization)

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer drake::drake)

add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test
    PRIVATE
        kinematics
)

add_executable(ompl_test ompl_test.cpp)
target_link_libraries(ompl_test
    PRIVATE
        planning
)

add_executable(trajopt_test trajopt_test.cpp)
target_link_libraries(trajopt_test
    PRIVATE
        optimization
)

add_executable(classic_motion_planning classic_motion_planning.cpp)
target_link_libraries(classic_motion_planning
    PRIVATE
        kinematics
        planning
        optimization
)

add_executable(motion_planning_refactor motion_planning_refactor.cpp)
target_link_libraries(motion_planning_refactor
    PRIVATE
        kinematics
)

add_executable(simulation simulation.cpp)
target_link_libraries(simulation
    PRIVATE
        kinematics
        planning
        optimization
)

add_executable(hardware_test hardware_test.cpp)
target_link_libraries(hardware_test
    PRIVATE
        hardware
)

add_executable(hardware_demo hardware_demo.cpp)
target_link_libraries(hardware_demo
    PRIVATE
        hardware
        kinematics
        planning
        optimization
)

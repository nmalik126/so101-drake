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

add_executable(gripper_visualizer gripper_visualizer.cpp)
target_link_libraries(gripper_visualizer
    PRIVATE 
        helpers
        kinematics
)

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

add_executable(gcs_test gcs_test.cpp)
target_link_libraries(gcs_test
    PRIVATE
        helpers
        optimization
)

add_executable(grasp_test grasp_test.cpp)
target_link_libraries(grasp_test
    PRIVATE
        helpers
        hardware
        kinematics
        planning
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

add_executable(binpick_demo binpick_demo.cpp)
target_link_libraries(binpick_demo
    PRIVATE
        helpers
        hardware
        kinematics
        planning
        optimization
)

add_executable(binpick_sim binpick_sim.cpp)
target_link_libraries(binpick_sim
    PRIVATE
        helpers
        kinematics
        planning
        optimization
)

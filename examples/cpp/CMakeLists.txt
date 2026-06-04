cmake_minimum_required(VERSION 3.10)
project(so101_drake_cpp)

find_package(drake CONFIG REQUIRED)

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer drake::drake)

add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test drake::drake)

add_executable(ompl_test ompl_test.cpp)
target_link_libraries(ompl_test drake::drake)

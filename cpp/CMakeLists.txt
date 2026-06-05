cmake_minimum_required(VERSION 3.10)
project(so101_drake_cpp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(drake CONFIG REQUIRED)
# find_package(ompl REQUIRED COMPONENTS base)
find_package(ompl REQUIRED)

message(STATUS "OMPL version: ${OMPL_VERSION}")
message(STATUS "OMPL include dirs: ${OMPL_INCLUDE_DIRS}")

add_executable(visualizer visualizer.cpp)
target_link_libraries(visualizer drake::drake)

add_executable(ik_test ik_test.cpp)
target_link_libraries(ik_test drake::drake)

add_executable(ompl_test ompl_test.cpp ompl_validity_checker.cpp)
target_include_directories(ompl_test PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(ompl_test PRIVATE drake::drake ${OMPL_LIBRARIES})

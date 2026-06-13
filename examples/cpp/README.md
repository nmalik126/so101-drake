# C++ Examples

This document outlines the C++ static libraries and executables for running manipulation tasks with the SO101 in Drake. Examples for both simulation-only and hardware-in-the-loop tasks are provided.

## Installation

1. Install the Drake and OMPL C++ libraries on your system

2. `cd` into `src/` and run the `build_fresh.sh` script

3. After the initial build, run the `build.sh` script to compile subsequent changes faster

## Static Libraries

The following static libraries are defined under `src/` and linked by the example programs:

- `hardware` - Defines interfaces to Drake LCM objects for interacting with the SO101 hardware.

- `kinematics` - Defines helper functions which run optimization-based Inverse Kinematics for bin-picking tasks.

- `planning` - Defines helper functions which run OMPL RRT-Connect given start and goal robot configurations.

- `optimization` - Defines helper functions which run Kinematic Trajectory Optimization given an initial guess from a sampling-based motion planner.

## Executables

The following example programs are available under `src/`:

- `visualizer` - Simple visualizer for the SO101 model.

- `ik_test` - Runs Inverse Kinematics given a sample grasp goal. Visualizes the resulting configuration.

- `ompl_test` - Runs RRT-Connect given a sample start and goal configuration. Visualizes the resulting collision-free C-space path.

- `trajopt_test` - Runs Kinematic Trajectory Optimization given a sample start and goal configuration and an initial guess motion plan (waypoints). Visualizes the resulting optimal collision-free motion plan.

- `classic_motion_planning` - Runs Inverse Kinematics, RRT-Connect, and Kinematic Trajectory Optimization in sequence given a sample grasp goal. Visualizes the resulting optimal collision-free motion plan.

- `simulation` - Generates three motion plans (for pick, place, and stow) according to `classic_motion_planning` for a sample bin-picking task. Simulates the result. 

- `hardware_test` - Sends commands to the SO101 hardware to open and close its gripper from its rest configuration. Visualizes the digital-twin in real time.

- `hardware_demo` - Runs the bin-picking task outlined in `simulation` on the SO101 hardware. Visualizes the digital-twin in real time.

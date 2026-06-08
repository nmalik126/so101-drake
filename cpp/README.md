# C++ Examples

This document outlines the C++ examples for SO101 manipulation tasks in Drake. Executables for both simulation-only and hardware-in-the-loop tasks are provided.

## Installation
1. Install the Drake C++ library on your system according to the [instructions](https://drake.mit.edu/installation.html)
2. Run the `build_fresh.sh` script
3. After the initial build, run the `build.sh` script to compile subsequent changes

## Executables

- `visualizer`: Simple visualizer for the SO101 model.

- `ik_test`: Runs Inverse Kinematics given a sample grasp goal. Visualizes the resulting configuration.

- `ompl_test`: Runs RRT-Connect given a sample start and goal configuration. Visualizes the resulting collision-free C-space path.

- `trajopt_test`: Runs Kinematic Trajectory Optimization given a sample start and goal configuration and an initial guess motion plan (waypoints). Visualizes the resulting optimal collision-free motion plan.

- `classic_motion_planning`: Runs Inverse Kinematics, RRT-Connect, and Kinematic Trajectory Optimization in sequence given a sample grasp goal. Visualizes the resulting optimal collision-free motion plan.

- `simulation`: Generates three motion plans (for pick, place, and stow) according to `classic_motion_planning` for a sample bin-picking task. Simulates the result. 

- `hardware_test`: Sends commands to the SO101 hardware to open and close its gripper from its rest configuration. Visualizes the digital-twin in real time.

- `hardware_demo`: Runs the bin-picking task outlined in `simulation` on the SO101 hardware. Visualizes the digital-twin in real time.

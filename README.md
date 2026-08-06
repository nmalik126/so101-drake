# so101-drake

End-to-end bin-picking system in C++ using the LeRobot SO-101 and Drake. 

## Usage

### Building

See build instructions at [`/examples/cpp/README.md`](./examples/cpp/README.md)

### Running

- Run [`simulation_demo`](./examples/cpp/src/simulation_demo.cpp) to perform bin-picking in simulation

- Run [`hardware_demo`](./examples/cpp/src/hardware_demo.cpp) to perform bin-picking on the SO-101 hardware

## System Overview

### Grasp Candidate Selection

Object grasp candidates are selected in one of two ways:

1. To generate grasp candidates directly from depth images, [Contact-GraspNet](https://github.com/NVlabs/contact_graspnet) is used. See [`grasp_test`](./examples/cpp/src/grasp_test.cpp) for component evaluation.

2. To generate grasp candidates from 3D models, ICP is used to perform model-based 6D object pose estimation. Grasp candidates can be computed offline for each 3D model. See [`realsense_pose_estimator`](./scripts/realsense_pose_estimator.py) for implementation.

### Inverse Kinematics

Inverse kinematics is performed using Drake's [`InverseKinematics`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html) library. A joint-centering cost was chosen as the optimization objective. See [`ik_test`](./examples/cpp/src/ik_test.cpp) for component evaluation.

### Collision-Free Motion Planning

Collision-free motion plans are computed in one of two ways:

1. If standard kinematic trajectory optimization will be used downstream, then motion plans are computed using [OMPL RRT-Connect](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html#gRRTC). See [`ompl_test`](./examples/cpp/src/ompl_test.cpp) for component evaluation.

2. If GCS trajectory optimization will be used downstream, then convex collision-free regions of C-space are computed using Drake's [`IrisNp`](https://drake.mit.edu/doxygen_cxx/group__planning__iris.html#ga9a3aba193bc960e38b6ae305e30dd13c) implementation. See [`gcs_test`](./examples/cpp/src/gcs_test.cpp) for component evaluation. 

### Kinematic Trajectory Optimization

Kinematic trajectory optimization is performed in one of two ways:

1. In the standard pipeline, Drake's [`KinematicTrajectoryOptimization`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html) library is used to compute (approximately) collision-free B-spline C-space trajectories. See [`trajopt_test`](./examples/cpp/src/trajopt_test.cpp) for component evaluation.

2. In the Graphs of Convex Sets pipeline, Drake's [`GcsTrajectoryOptimization`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_gcs_trajectory_optimization.html) library is used to compute collision-free Bezier-curve C-space trajectories. See [`gcs_test`](./examples/cpp/src/gcs_test.cpp) for component evaluation. 

### Control

C-space trajectories are executed in one of two ways:

1. In simulation, Drake's [`InverseDynamicsController`](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_inverse_dynamics_controller.html) implementation is used. See [`simulation_demo`](./examples/cpp/src/simulation_demo.cpp) for component evaluation.

2. On real hardware, the default Feetech PD position controller is used. A PWM-based approximate force controller is in progress, which will facilitate higher trajectory tracking precision and compliant control on hardware. 

## Additional Documentation

- See [`URDF.md`](./docs/URDF.md) for derivation of an improved SO-101 URDF optimized for Drake. Involves configuration/tuning of collision geometries and kinematic/dynamic parameters.

- See [`Calibration.md`](./docs/Calibration.md) for derivation of an improved SO-101 motor calibration procedure. Reduces end-effector error on real hardware from ~20mm to ~2mm.

- See [`Hardware.md`](./docs/Hardware.md) for an explanation of how to visualize and control the SO-101 hardware in real-time using Drake.


## Acknowledgements

Many manipulation concepts and system design patterns implemented in this project were derived from Prof. Russ Tedrake's [manipulation course](https://manipulation.csail.mit.edu/). Portions of the Python example code are derived from the [Drake tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials) and [manipulation exercises](https://github.com/RussTedrake/manipulation/tree/master/manipulation/exercises); see the course for relevant technical background. The C++ bin-picking system, SO-101 hardware integration, improved URDF, and motor calibration procedure are original work.

# so101-drake
Manipulation experiments in C++ and Python with the [SO-101 arm](https://huggingface.co/docs/lerobot/so101) in Drake

Most examples derived from [drake tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials) and [manipulation exercies](https://github.com/RussTedrake/manipulation/tree/master/manipulation/exercises). See Prof. Russ Tedrake's [manipulation course](https://manipulation.csail.mit.edu/) for a relevant technical background

Contributions primarily relate to integration with the SO-101 arm:
- See [`/examples`](./examples/) for both C++ and Python implementations of various manipulation tasks with the SO-101 in Drake
- See [`/docs/URDF.md`](./docs/URDF.md) for an improved SO-101 URDF optimized for Drake
- See [`/docs/Calibration.md`](./docs/Calibration.md) for an improved SO-101 motor calibration procedure
- See [`/docs/Hardware.md`](/docs/Hardware.md) for using Drake to visualize and control the SO-101 hardware

## Repository Index

- `/calibrations` - JSON files and NumPy arrays for motor calibration
- `/docs` - Writeups for core tasks and findings
- `/examples` - C++ and Python implementations for example tasks with the SO-101 in Drake
- `/lcmdefs` - LCM messages for hardware <-> Drake communications
- `/media` - Images and GIFs for READMEs
- `/models` - SDFs and URDFs for robot and objects
- `/scenarios` - YAMLs for Drake scenarios
- `/scripts` - Helper functions for interfacing with hardware 

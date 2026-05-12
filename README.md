# so101-drake
Manipulation experiments with the [SO-101 arm](https://huggingface.co/docs/lerobot/so101) in Drake

Most examples derived from [drake tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials) and [manipulation exercies](https://github.com/RussTedrake/manipulation/tree/master/manipulation/exercises). See Prof. Russ Tedrake's [manipulation course](https://manipulation.csail.mit.edu/) for a relevant technical background

Contributions primarily relate to integration with the SO-101 arm:
- See [`/examples`](./examples/) for implementations of various manipulation tasks with the SO-101 in Drake
- See [`/docs/URDF.md`](./docs/URDF.md) for an improved SO-101 URDF optimized for Drake
- See [`/docs/Calibration.md`](./docs/Calibration.md) for an improved SO-101 motor calibration procedure

## Installation

Tested on Ubuntu 22.04 with Python 3.12

1. Install conda

2. Create environment \
`conda create -n so101-drake python=3.12`

3. Activate environment \
`conda activate so101-drake`

4. Install manipulation package \
`pip install manipulation[all] --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/`

5. Install lerobot dependencies \
`sudo apt-get install cmake build-essential python3-dev pkg-config libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libswscale-dev libswresample-dev libavfilter-dev`

6. Install lerobot package \
`pip install 'lerobot[all]'`

7. Install lcm \
`pip install lcm`

Installation instructions retrieved from these links:

- [Drake](https://manipulation.csail.mit.edu/drake.html#section3)
- [Lerobot](https://huggingface.co/docs/lerobot/installation)
- [LCM](https://lcm-proj.github.io/lcm/content/install-instructions.html#installing-lcm)

## Usage

- Use the conda environment created in [Installation](#installation) as the Python kernel for Jupyter notebooks under `/examples` and scripts under `/scripts`

- Before using the scenarios under `/scenarios`, change the model absolute file paths to your own

- Before running on hardware, follow the revised calibration procedure described in [`/docs/Calibration.md`](./docs/Calibration.md)

## Repository Index

- `/calibrations` - JSON files and NumPy arrays for motor calibration
- `/docs` - Writeups for core tasks and findings
- `/examples` - Jupyter notebooks for example tasks with the SO-101 in Drake
- `/lcmdefs` - LCM messages for hardware <-> Drake communications
- `/media` - Images and GIFs for READMEs
- `/models` - SDFs and URDFs for robot and objects
- `/scenarios` - YAMLs for Drake scenarios
- `/scripts` - Helper functions for interfacing with hardware 

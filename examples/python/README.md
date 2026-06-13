# Python Examples

This document outlines the Python examples for SO101 manipulation tasks in Drake. Executables for both simulation-only and hardware-in-the-loop tasks are provided.

## Installation

Tested on Ubuntu 22.04 with Python 3.12. 

Warning: If Drake is already installed on your system by another method (e.g. from source or apt package), that installation may overlap with the pip installation described below. It is simplest to only maintain one installation of Drake per system.

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

7. Install lcm package \
`pip install lcm`

8. Install open3d package \
`pip install open3d`

Installation instructions retrieved from these links:

- [Drake](https://manipulation.csail.mit.edu/drake.html#section3)
- [Lerobot](https://huggingface.co/docs/lerobot/installation)
- [LCM](https://lcm-proj.github.io/lcm/content/install-instructions.html#installing-lcm)

## Usage

- Use the conda environment created in [Installation](#installation) as the Python kernel for Jupyter notebooks under `/examples` and scripts under `/scripts`

- Before using the scenarios under `/scenarios`, change the model absolute file paths to your own

- Before running on hardware, follow the revised calibration procedure described in [`/docs/Calibration.md`](./docs/Calibration.md)

## Notebooks

- `visualizer.ipynb` - Simple visualizer for SO101 model

- `joint_limits.ipynb` - Derivation of SO101 joint limits

- `state_viewer.ipynb` - View current robot state from SO101 hardware, visualize digital twin 

- `diffik_pick.ipynb` - Perform pick-and-place task using Diff-IK in simulation

- `hardware.ipynb` - Perform pick-and-place task using Diff-IK on SO101 hardware, visualize digital twin

- `motion_planning_test.ipynb` - Perform bin-picking task using classical motion planning on SO101 hardware, visualize digital twin

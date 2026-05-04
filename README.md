# so101-drake
Manipulation experiments with the SO-101 arm in Drake

Most examples derived from [manipulation exercies](https://github.com/RussTedrake/manipulation/tree/master/manipulation/exercises) and [drake tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials)

Primary contributions involve integration with the SO-101 arm

## Installation

Instructions derived from [here](https://manipulation.csail.mit.edu/drake.html#section3)

Tested on Ubuntu 22.04 with Python 3.12

1. Install conda

2. Create environment \
`conda create -n so101-drake python=3.12`

3. Activate environment \
`conda activate so101-drake`

4. Install manipulation package \
`pip install manipulation[all] --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/`

## Usage

Before using the scenarios under `/scenarios`, change the model absolute file paths to your own  

## Repository Index

- `/examples` - Jupyter notebooks demonstrating example tasks in Drake with the SO-101
- `/models` - SDFs and URDFs for robot and objects
- `/scenarios` - YAMLs for Drake scenarios
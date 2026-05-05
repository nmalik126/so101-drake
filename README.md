# so101-drake
Manipulation experiments with the SO-101 arm in Drake

Most examples derived from [drake tutorials](https://github.com/RobotLocomotion/drake/tree/master/tutorials) and [manipulation exercies](https://github.com/RussTedrake/manipulation/tree/master/manipulation/exercises). See Prof. Russ Tedrake's [manipulation course](https://manipulation.csail.mit.edu/) for relevant technical background

Contributions primarily relate to integration with the SO-101 arm. See [here](models/SO101/README.md) for derivation of a Drake-compatible SO-101 model

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

Use the conda environment created in [Installation](#installation) as the Python kernel for Jupyter notebooks under `/examples`

Before using the scenarios under `/scenarios`, change the model absolute file paths to your own

## Repository Index

- `/examples` - Jupyter notebooks demonstrating example tasks with the SO-101 in Drake
- `/media` - Images and GIFs used in READMEs
- `/models` - SDFs and URDFs for robot and objects
- `/scenarios` - YAMLs for Drake scenarios

# SO-101 Drake Model

This document describes how the Drake-compatible URDF `so101_new_calib_drake.urdf` was prepared

## Source URDF

The source SO-101 URDF `so101_new_calib.urdf` and associated model assets can be obtained from [the project page](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101). They are copied here for convenience.

The source URDF is not compatible with Drake as-is. To make it compatible, run the `make_drake_compatible_model` utility as recommended on the [Drake troubleshooting page](https://drake.mit.edu/troubleshooting.html)

1. Activate environment \
`conda activate so101-drake`

2. Run utility \
`python -m manipulation.make_drake_compatible_model so101_new_calib.urdf {your filename here}.urdf`

## Modifications

Although the source URDF is parseable by Drake after running `make_drake_compatible_model`, a few more inclusions are necessary for it to be useable for manipulation in practice

### Enable Hydroelastic Contact

lorem ipsum

### Configure Reflected Inertia

lorem ipsum
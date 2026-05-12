# SO-101 Drake Hardware Integration

This document describes the integration required to visualize and control the SO-101 hardware using Drake

## Overview

As a motivating example, a LEGO block with known shape and pose was picked and placed into a bin.

For simplicity, the motion control algorithm chosen was unconstrained Diff-IK. Task keyframes were manually chosen so as to avoid joint limits and singularities. The closed-loop Diff-IK policy was combined with simple open-loop joint-space helper trajectories for opening/closing the gripper finger and moving the arm from "rest" position to "ready" position and back.

|  |  |  |
| :---: | :---: | :---: |
| <img src="../media/hw_pick_sim.gif" style="width: 55%; height: auto;"> | <img src="../media/hw_pick_digital_twin.gif" style="width: 60%; height: auto;"> | <img src="../media/hw_pick_real.gif" style="width: 100%; height: auto;"> |
| *Offline Simulation* | *Online Digital Twin* | *Real Hardware* |

## Drake Integration

lorem ipsum

## LeRobot Integration

lorem ipsum
import numpy as np
import json

from pydrake.all import (
    Parser,
    MultibodyPlant
)

# 1. Get true joint limits
plant = MultibodyPlant(time_step=1e-3)
Parser(plant).AddModels("../models/SO101/so101_new_calib_drake.urdf")
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base_link"))
plant.Finalize()
joint_lower_limits = plant.GetPositionLowerLimits()
joint_upper_limits = plant.GetPositionUpperLimits()

# 2. Get calibration joint limits
calib_filename = "my_awesome_follower_arm"
with open(f"../calibrations/{calib_filename}.json", 'r') as f:
    calib_dict = json.load(f)
joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
joint_range_mins = np.array([calib_dict[joint_name]['range_min'] for joint_name in joint_names], np.int16)
joint_range_maxs = np.array([calib_dict[joint_name]['range_max'] for joint_name in joint_names], np.int16)
joint_lowers = (joint_range_mins - 2048) * (2 * np.pi / 4096)
joint_uppers = (joint_range_maxs - 2048) * (2 * np.pi / 4096)

# 3. Compute biases
biases = ((joint_lower_limits - joint_lowers) + (joint_upper_limits - joint_uppers)) / 2
print(biases)

with open(f'../calibrations/{calib_filename}_biases.npy', 'wb') as f:
    np.save(f, biases)
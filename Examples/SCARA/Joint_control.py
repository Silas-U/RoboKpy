"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:
http://www.apache.org/licenses/LICENSE-2.0
"""

from Robokpy import Init_Model
from Model import DHModel
import numpy as np


# Initialize Cobra600 Robot
model = DHModel.get_model("Cobra600")
robot = Init_Model(
    model,
    robot_name="Cobra600",
    twist_in_rads=True,
    use_jnt_lim=True,
    plt_model=True
)

# Set Joint Limits
robot.model.set_joint_limits({
    "min": {"j1": -np.pi, "j2": -np.pi/2, "j3": 0.04, "j4": -np.pi},
    "max": {"j1": np.pi,  "j2":  np.pi/2, "j3": 0.06, "j4":  np.pi}
})

# Define Joint Poses
joint_poses = [
    [90, 0, 0.05, 0],
    [0, 90, 0.05, 0],
    [-90, 90, 0.05, 0],
    [0, 0, 0.05, 0],
    [0, 0, 0.2, 0],
    [0, 0, 0.2, 90],
    [90, 0, 0.05, 0],
]

# Generate Joint-Space Trajectory
robot.traj.traj_type("qu")
joint_trajectory = robot.traj.joint_control(joint_poses, n_samples=20, rads=False)

# Visualize Robot Motion
robot.mviz.scale_viz(0.5, 0.05)
robot.mviz.plot(
    joint_trajectory,
    show_path=True,
    show_via_points=True,
    repeat=True
)

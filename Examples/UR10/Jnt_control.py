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


# Initialize UR10 Robot
model = DHModel.get_model("UR10")
robot = Init_Model(
    model,
    robot_name="UR10",
    plt_model=True
)

# Define Joint-Space Waypoints (in radians)
j_poses = [
    [np.deg2rad(90), np.deg2rad(45), np.deg2rad(90), np.deg2rad(135), np.deg2rad(90), np.deg2rad(90)],
    [np.deg2rad(-90), np.deg2rad(45), np.deg2rad(90), np.deg2rad(135), np.deg2rad(90), np.deg2rad(90)],
    [np.deg2rad(-90), np.deg2rad(90), np.deg2rad(-90), np.deg2rad(90), np.deg2rad(90), np.deg2rad(90)],
    [np.deg2rad(-90), np.deg2rad(90), 0, 0, 0, 0],
    [np.deg2rad(90), np.deg2rad(90), 0, np.deg2rad(90), 0, 0],
    [np.deg2rad(90), np.deg2rad(90), 0, np.deg2rad(90), 0, np.deg2rad(90)],
    [np.deg2rad(90), np.deg2rad(45), np.deg2rad(90), np.deg2rad(135), np.deg2rad(90), np.deg2rad(90)],
]

# Generate Trajectory
robot.traj.traj_type("qu")
trajectory = robot.traj.joint_control(j_poses, n_samples=20, rads=True)

# Visualize Trajectory
robot.mviz.scale_viz(0.5, 0.07)
robot.mviz.plot(
    trajectory,
    show_path=False,
    show_via_points=False,
    repeat=True
)

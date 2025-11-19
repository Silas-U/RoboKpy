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

# Initialize Puma561 robot
model = DHModel.get_model("Puma561")
robot = Init_Model(model, robot_name="Puma561", plt_model=True)

# Define joint poses [rad] for trajectory
joint_poses = [
    [0, 0, 0, 0, 0, 0],
    [np.pi/2, 0, 0, np.pi, 0, np.pi/2],
    [np.pi/2, np.pi/2, 0, np.pi, 0, np.pi/2],
    [np.pi/2, np.pi/2, np.pi/2, np.pi, 0, np.pi/2],
    [-np.pi/2, np.pi/2, np.pi/2, np.pi, 0, np.pi/2],
    [-np.pi/2, np.pi/2, 0, np.pi, 0, np.pi/2],
    [0, np.pi/2, 0, np.pi, np.pi/2, np.pi/2],
    [0, 0, 0, 0, 0, 0],
]

# Set trajectory type to quintic
robot.traj.traj_type('qu')

# Generate trajectory in joint space
trajectory = robot.traj.joint_control(joint_poses, n_samples=20, rads=True)

# Visualize the robot motion
robot.mviz.scale_viz(0.35, 0.04)
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_via_points=True,
    repeat=True
)

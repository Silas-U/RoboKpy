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

# Define center of circular trajectory [x, y, z, roll, pitch, yaw]
center = [0.2, 0.0, 0.1, np.pi, 0.0, -np.pi/6]

# Generate circle trajectory in task space
circle_waypoints = robot.traj.create_circle_traj(radius=0.05, cent=center)

# Convert waypoints to joint angles
joint_trajectory = robot.traj.wayp_to_joint_angle(circle_waypoints)

# Visualization settings
robot.mviz.scale_viz(0.25, 0.04)
robot.mviz.show_target([center], axis_length=0.0, sc=True)

# Plot the trajectory
robot.mviz.plot(
    joint_trajectory,
    show_path=True,
    show_via_points=False,
    repeat=True
)

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


# Initialize Cobra600 Robot
model = DHModel.get_model("Cobra600")
robot = Init_Model(model, robot_name="Cobra600", twist_in_rads=True, plt_model=True)

# Define Circular Trajectory
center = [0.2, 0.1, 0.2, 3.14159265, 0.0, 1.57079633]  # Center of the circle [x, y, z, roll, pitch, yaw]
radius = 0.07

# Generate circular trajectory in task space
circle_waypoints = robot.traj.create_circle_traj(radius=radius, cent=center)

# Convert waypoints to joint angles (masking only XYZ and yaw)
joint_trajectory = robot.traj.wayp_to_joint_angle(circle_waypoints, xyz_mask=[1, 1, 1, 0, 0, 1])

# Visualization
robot.mviz.scale_viz(0.35, 0.04)               # Scale robot model for visualization
robot.mviz.show_target([center], axis_length=0.02, sc=True)  # Show target center
robot.mviz.plot(
    joint_trajectory,
    show_path=True,
    show_via_points=True,
    repeat=True
)

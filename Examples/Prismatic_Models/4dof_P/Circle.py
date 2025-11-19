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


# Initialize Cylindrical robot model
model = DHModel.get_model("Cylindrical")
rb = Init_Model(
    model,
    robot_name="Cylindrical",
    plt_model=True
)

# Define circle center in task space [x, y, z, roll, pitch, yaw]
center = [0.30, 0.20, 0.10, 0.0, 0.0, 1.57079633]

# Generate circular trajectory and map to joint angles
circle = rb.traj.create_circle_traj(radius=0.10, cent=center)
jt = rb.traj.wayp_to_joint_angle(circle, xyz_mask=[1, 1, 1, 0, 0, 0])

# Visualization setup
rb.mviz.scale_viz(0.40, 0.05)
rb.mviz.show_target([center], axis_length=0.05, sc=True)

# Plot motion
rb.mviz.plot(
    jt,
    show_path=True,
    show_via_points=False,
    show_final_pose=True,
    show_joint_label=True,
    repeat=True
)

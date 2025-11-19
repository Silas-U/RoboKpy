"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from Robokpy import Init_Model
from Model import DHModel

# Initialize 6-DOF robot model
model = DHModel.get_model('6dof')
rb = Init_Model(model, robot_name='6dof', twist_in_rads=True, plt_model=True)

# Define the circle trajectory center point (XYZ + RPY)
center = [
    0.2,        # x-position
    0.0,        # y-position
    0.1,        # z-position
    3.14159265, # roll
    0.0,        # pitch
    -0.52359878 # yaw
]

# Generate circular trajectory and convert to joint angles
circle = rb.traj.create_circle_traj(radius=0.05, cent=center)
joint_traj = rb.traj.wayp_to_joint_angle(circle)

# Configure visualization and plot the motion
rb.mviz.scale_viz(0.2, 0.03)
rb.mviz.show_target([center], 0.0, sc=True)
rb.mviz.plot(
    joint_traj,
    show_path=True,
    show_via_points=False,
    show_final_pose=True,
    repeat=True
)

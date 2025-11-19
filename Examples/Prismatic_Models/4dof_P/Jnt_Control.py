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

# Define joint-space waypoints
# Format: [theta1, prismatic_joint1, prismatic_joint2, theta2]
j_poses = [
    [0,   0.00, 0.20, 0.0],
    [90,  0.30, 0.45, 0.2],
    [0,   0.00, 0.20, 0.0],
    [90,  0.30, 0.45, 0.2],
    [0,   0.00, 0.20, 0.0],
]

# Configure trajectory generation
rb.traj.traj_type("qu")
wp = rb.traj.joint_control(
    j_poses,
    n_samples=20,
    rads=False
)

# Robot motion visualization
rb.mviz.scale_viz(0.6, 0.09)
rb.mviz.plot(
    wp,
    show_path=True,
    show_via_points=True,
    repeat=True
)

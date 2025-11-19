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

# Initialize 6DOF robot model
model = DHModel.get_model('6dof')
rb = Init_Model(model, robot_name='6dof', twist_in_rads=True, plt_model=True)

# Joint-space trajectory waypoints
j_poses = [
    [0,   0,   0,   0,   0,  0],
    [0,  90,   0,   0,   0,  0],
    [0,  90, -90, 180,   0,  0],
    [-90, 90, -90, 180,  0,  0],
    [-90, 90, -90, 180,  0, 90],
    [0,  90, -90, 180,   0,  0],
    [0,   0,   0,   0,   0,  0],
]

# Trajectory generation
rb.traj.traj_type('qu') # quintic-based
wp = rb.traj.joint_control(j_poses, n_samples=20)

# Visualization
rb.mviz.scale_viz(0.23, 0.03)
rb.mviz.plot(
    wp,
    show_path=True,
    show_via_points=False,
    show_final_pose=True,
    repeat=True
)

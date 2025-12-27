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


model = DHModel.get_model("4dof")
robot = Init_Model(model, robot_name="4dof", plt_model=True)

joint_poses = [
    [0, 120, -90, -90],
    [0, 0, 0, 0],
    [0, 120, -90, -90],
    [90, 120, -90, -90],
    [0, 120, -90, -90],
    [-90, 120, -90, -90],
    [0, 120, -90, -90],
]

# Generate Joint-Space Trajectory
robot.traj.traj_type("qu")
joint_trajectory = robot.traj.joint_control(joint_poses, n_samples=20, rads=False)
robot.plotter.plot_traj(plot_type='traj', selected_plot='all')
robot.plotter.plot_traj(plot_type='jc', selected_plot='all')

# Visualize Robot Motion
robot.mviz.scale_viz(3.5, 0.7)
robot.mviz.plot(
    joint_trajectory,
    show_path=True,
    repeat=True,
    show_joint_label=True,
    show_via_points=True
)



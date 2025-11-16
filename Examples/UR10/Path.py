
"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from robokpy import Init_Model
from Models.Model import DHModel

model = DHModel.get_model('UR10')
robot = Init_Model(model, robot_name='UR10', plt_model=True)

waypoints = [
    [0.30,  0.00, 0.10, 180, 0, 0],
    [0.25,  0.02, 0.11, -180, 0, 0],
    [0.20,  0.00, 0.13, -180, 0, 0],
    [0.15, -0.02, 0.11, -180, 0, 0],
    [0.10,  0.00, 0.10, -180, 0, 0],
    [0.15, -0.02, 0.11, -180, 0, 0],
    [0.20,  0.00, 0.13, -180, 0, 0],
    [0.25,  0.02, 0.11, -180, 0, 0],
    [0.30,  0.00, 0.10, -180, 0, 0],
]

robot.model.set_eular_in_deg(True)
robot.traj.set_traj_time(5.0)
robot.traj.scale_waypoint_vel = 0.0
robot.traj.traj_type('qu')
trajectory = robot.traj.create_trajectory(waypoints, traj_method='ts', n_samples=20)
print("waypoint_velocities:\n", robot.traj.get_waypoint_velocities())
robot.plotter.plot_traj(plot_type='vel', selected_plot='all')

robot.mviz.scale_viz(0.3, 0.05)
robot.mviz.show_target(waypoints, 0.04, sc=True)
robot.mviz.plot(trajectory, show_path=True, show_via_points=False, show_joint_label=True, repeat=True)

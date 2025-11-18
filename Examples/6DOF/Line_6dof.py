"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from robokpy import Init_Model
from Model import DHModel

model = DHModel.get_model('6dof')
robot = Init_Model(model, robot_name='6dof', twist_in_rads=True, plt_model=True)

waypoints = [
    [0.2, 0.1, 0.1,  3.14159265,  0.,  -0.52359878],
    [0.2, -0.15, 0.1,  3.14159265,  0., -0.52359878],
    [0.2, 0.1, 0.1,  3.14159265,  0.,  -0.52359878],
]

robot.traj.traj_type('qu')
trajectory = robot.traj.create_trajectory(waypoints, traj_method='ts', n_samples=20)
robot.plotter.plot_traj(plot_type='vel', selected_plot='all')

robot.mviz.scale_viz(0.17, 0.02)
robot.mviz.show_target(waypoints, sc=True)
robot.mviz.plot(trajectory, show_path=True, show_via_points=False, repeat=True)

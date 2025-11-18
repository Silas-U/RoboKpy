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
import numpy as np

model = DHModel.get_model('Cobra600')
robot = Init_Model(model, robot_name='Cobra600', twist_in_rads=True, use_jnt_lim=True, plt_model=True)

robot.model.set_joint_limits(
    {
        "min": { "j1": -np.pi, "j2": -np.pi/2, "j3": 0.0, "j4": -np.pi },
        "max": { "j1": np.pi,"j2": np.pi/2, "j3":0.5,"j4": np.pi }
    }
)

waypoints = [
    [0.35,  0.1, 0.3,   3.14159265, 0.,         1.57079633],
    [0.15,  0.1, 0.3,   3.14159265, 0.,         1.30899694],
    [0.35,  0.1, 0.3,   3.14159265, 0.,         1.57079633],
]

robot.model.set_eular_in_deg(True)
robot.traj.set_traj_time(5.0)
robot.traj.traj_type('qu')
trajectory = robot.traj.create_trajectory(waypoints, traj_method='ts', n_samples=20, xyz_mask=[1, 1, 1, 0, 0, 0])
robot.plotter.plot_traj(plot_type='vel', selected_plot='all')

robot.mviz.scale_viz(0.45, 0.05)
robot.mviz.show_target(waypoints, 0.02, sc=True)
robot.mviz.plot(trajectory, show_path=True, show_via_points=True, show_joint_label=True, repeat=True)

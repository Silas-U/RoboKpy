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

model = DHModel.get_model('UR10')
rb = Init_Model(model, robot_name='UR10', plt_model=True)

j_poses = [
    [90*np.pi/180, 45*np.pi/180, 90*np.pi/180, 135*np.pi/180, 90*np.pi/180, 90*np.pi/180],
    [-90*np.pi/180, 45*np.pi/180, 90*np.pi/180, 135*np.pi/180, 90*np.pi/180, 90*np.pi/180],
    [-90*np.pi/180, 90*np.pi/180, -90*np.pi/180, 90*np.pi/180, 90*np.pi/180, 90*np.pi/180],
    [-90*np.pi/180, 90*np.pi/180, 0, 0*np.pi/180, 0, 0],
    [90*np.pi/180, 90*np.pi/180, 0, 90*np.pi/180, 0, 0],
    [90*np.pi/180, 90*np.pi/180, 0, 90*np.pi/180, 0, 90*np.pi/180],
    [90*np.pi/180, 45*np.pi/180, 90*np.pi/180, 135*np.pi/180, 90*np.pi/180, 90*np.pi/180],
]

rb.traj.traj_type('qu')
wp = rb.traj.joint_control(j_poses, n_samples=20, rads=True)
rb.plotter.plot_traj(plot_type='xyz', selected_plot='all', projection='2d')

rb.mviz.scale_viz(0.5, 0.07)
rb.mviz.plot(wp, show_path=False, show_via_points=False, repeat=True)

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

model = DHModel.get_model('2dof')
rb = Init_Model(model, robot_name='2dof', plt_model=True)

mdl_lst = DHModel.list_models()
print(mdl_lst)

j_poses = [
    [0,  0],
    [-90,  0],
    [-90,  90],
    [0,  0],
]

rb.traj.traj_type('qu')
wp = rb.traj.joint_control(j_poses, n_samples=20)
rb.plotter.plot_traj(plot_type='xyz', selected_plot='all', projection='2d')

rb.mviz.scale_viz(0.5, 0.05)
rb.mviz.plot(wp, show_path=True, show_via_points=True, repeat=True)

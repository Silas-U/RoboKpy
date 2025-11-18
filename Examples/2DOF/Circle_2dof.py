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

model = DHModel.get_model('2dof')
rb = Init_Model(model, robot_name='2dof', plt_model=True)

center = [0.3,        0.2,        0.2,        0.,         0.,         1.57079633]
circle = rb.traj.create_circle_traj(radius=0.09, cent=center)
jt = rb.traj.wayp_to_joint_angle(circle, xyz_mask=[1, 1, 1, 0, 0, 0])

rb.mviz.scale_viz(0.35, 0.05)
rb.mviz.show_target([center], 0.0, sc=True)
rb.mviz.plot(jt, show_path=True, show_via_points=False, show_final_pose=True, repeat=True)
 
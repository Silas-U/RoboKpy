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
import time

robot_model = DHModel.get_model("Puma560")
rb = Init_Model(robot_model, robot_name="Puma560", twist_in_rads=False, plt_model=True)

start = time.time()
target_pose = [ 0.59630552, -0.15, -0.01435103, -0, 1.57078531, -0. ]
solution = rb.ik.solve(target_pose, max_iter=100)
elapsed = time.time() - start
print(f"IK solved in {elapsed*1000:.3f} ms")
print("IK solution :")
print(solution)

# Show dh model in plot
rb.mviz.scale_viz(0.59, 0.05)
rb.mviz.show_dh_model(solution)

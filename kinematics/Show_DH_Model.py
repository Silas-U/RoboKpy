"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from Robokpy import Init_Model
from Model import DHModel


# Initialize Robot Model
robot_model = DHModel.get_model("Puma560")
rb = Init_Model(robot_model, robot_name="Puma560", plt_model=True)

# Define joint configurations
elbow_up = [0, 0.7854, 3.1416, 0, 0.7854, 0]
elbow_down = [0, -0.8335, 0.0940, 0, -0.8312, 0]

# Forward Kinematics
rb.fk.compute(elbow_up, rads=True)
T = rb.fk.get_htm()  # 4x4 homogeneous transform
print("End-effector transform:\n", T, '\n')

target_pose = rb.fk.get_target()  # [x, y, z, roll, pitch, yaw]
print("FK target pose:", target_pose, '\n')

# Jacobian
J = rb.jac.compute()
print("Jacobian Matrix:\n", J, '\n')

# Inverse Kinematics
solution = rb.ik.solve(target_pose)
print("IK solution (rads):", solution, '\n')

# Visualize DH model in 3D
rb.mviz.scale_viz(0.59, 0.05)
rb.mviz.show_dh_model(solution)

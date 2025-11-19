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


# Initialize Puma560 Robot
robot_model = DHModel.get_model("Puma560")
rb = Init_Model(robot_model, robot_name="Puma560", twist_in_rads=False)

# Compute Forward Kinematics
joint_conf = [0, 0.7854, 3.1416, 0, 0.7854, 0]  # Joint angles in radians
rb.fk.compute(joint_conf, rads=True)

# Display End-Effector Information
print("*** Target (XYZ) ***")
print(rb.fk.get_target_xyz(), '\n')  # Position of end-effector

print("*** Euler Angles (Roll, Pitch, Yaw) ***")
print(rb.fk.get_target_rpy(), '\n')  # Orientation of end-effector

print("*** Target (XYZ + Roll, Pitch, Yaw) ***")
print(rb.fk.get_target(), '\n')  # Complete pose

print("*** Homogeneous Transformation Matrix ***")
print(rb.fk.get_htm())  # 4x4 transformation matrix

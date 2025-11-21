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
joint_conf = [0,  0,  0,  180, 90, 0]  # Joint angles in radians
rb.fk.compute(joint_conf)

# Display End-Effector Information
print("Target:[px,py,pz]")
print(rb.fk.get_target_xyz(), '\n')

print("Quaternions:[qx,qy,qz,qw]")
print(rb.fk.get_target_quart(), '\n')

print("Target:[px,py,pz, qx,qy,qz,qw]")
print(rb.fk.get_target(), '\n')

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

# Initialize Puma560 model
robot_model = DHModel.get_model("Puma560")
rb = Init_Model(robot_model, robot_name="Puma560", twist_in_rads=False)

# Compute forward kinematics for a given joint configuration
joint_angles = [0, 0.7854, 3.1416, 0, 0.7854, 0]  # in radians
rb.fk.compute(joint_angles, rads=True)

# Display the homogeneous transformation matrices for each link
rb.fk.show_tranformations()

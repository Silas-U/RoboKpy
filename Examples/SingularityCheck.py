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

# Initialize Puma561 Robot
model = DHModel.get_model("Puma561")
rb = Init_Model(model, robot_name="Puma561")

# Define Joint Configuration [deg]
joint_conf = [90, 45, -60, 20, 0, 90]

# Compute Forward Kinematics
rb.fk.compute(joint_conf)

# Check for Singularities
rb.jac.singular_conf_check()

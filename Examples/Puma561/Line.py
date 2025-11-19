"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:
http://www.apache.org/licenses/LICENSE-2.0
"""

from Robokpy import Init_Model
from Model import DHModel

# Initialize Puma561 robot
model = DHModel.get_model("Puma561")
robot = Init_Model(model, robot_name="Puma561", plt_model=True)

# Define task-space waypoints [x, y, z, roll, pitch, yaw]
waypoints = [
    [0.2, 0.1, 0.1, 3.14159265, 0., -0.52359878],
    [0.2, -0.2, 0.1, 3.14159265, 0., -0.52359878],
    [0.2, 0.1, 0.1, 3.14159265, 0., -0.52359878],
]

# Configure trajectory parameters
robot.traj.set_traj_time(5.0)    # Total trajectory time (seconds)
robot.traj.traj_type('qu')       # Quintic trajectory

# Generate trajectory
trajectory = robot.traj.create_trajectory(
    waypoints, traj_method='ts', n_samples=20
)

# Visualization setup
robot.mviz.scale_viz(0.3, 0.04)                   # Scale robot visualization
robot.mviz.show_target(waypoints, 0.0, sc=True)  # Show target waypoints
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_via_points=True,
    show_joint_label=True,
    repeat=True
)

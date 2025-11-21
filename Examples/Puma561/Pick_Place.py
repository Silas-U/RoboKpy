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


# Initialize Puma561 Robot
model = DHModel.get_model("Puma561")
robot = Init_Model(model, robot_name="Puma561", plt_model=True)

# Define Task-Space Waypoints
# Format: [x, y, z, rx, ry, rz]
waypoints = [
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 1
    [0.20, -0.10, 0.07, 180, 0, -30],  # Target 2
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 3
    [0.20,  0.10, 0.15, 180, 0, -30],  # Target 4
    [0.20,  0.10, 0.07, 180, 0, -90],  # Target 5
    [0.20,  0.10, 0.15, 180, 0, -30],  # Target 6
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 1
]

# Trajectory Configuration
robot.model.set_eular_in_deg(True)    # Use degrees for Euler angles
robot.traj.traj_type('qu')           # Quintic trajectory

# Generate trajectory
trajectory = robot.traj.create_trajectory(
    waypoints, traj_method='ts', n_samples=10
)

# Visualization
robot.mviz.scale_viz(0.3, 0.04)                # Scale robot for visualization
robot.mviz.show_target(waypoints, 0.04, sc=True)  # Show waypoints in 3D
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_joint_label=True,
    repeat=True
)

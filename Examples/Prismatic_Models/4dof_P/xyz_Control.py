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


# Initialize Cylindrical robot
model = DHModel.get_model("Cylindrical")
robot = Init_Model(
    model,
    robot_name="Cylindrical",
    plt_model=True
)

# Define waypoints in task space [x, y, z, roll, pitch, yaw]
waypoints = [
    [0.10, 0.10, 0.10, 0.0, 0.0, 0.0],
    [0.20, 0.20, 0.10, 0.0, 0.0, 0.0],
    [0.30, 0.10, 0.10, 0.0, 0.0, 0.0],
    [0.10, 0.10, 0.10, 0.0, 0.0, 0.0],
]

# Robot and trajectory configuration
robot.model.set_eular_in_deg(True)
robot.traj.scale_waypoint_vel = 0.0
robot.traj.traj_type("qu")

# Generate trajectory in task space
trajectory = robot.traj.create_trajectory(
    waypoints,
    traj_method="ts",
    n_samples=20,
    xyz_mask=[1, 1, 1, 0, 0, 0]
)

# Print waypoint velocities
print("Waypoint velocities:\n", robot.traj.get_waypoint_velocities())

# Visualization setup
robot.mviz.scale_viz(0.25, 0.05)
robot.mviz.show_target(waypoints, axis_length=0.0, sc=True)

# Plot robot motion along trajectory
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_via_points=True,
    show_joint_label=True,
    repeat=True
)

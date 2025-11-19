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


# Initialize UR10 Robot
model = DHModel.get_model("UR10")
robot = Init_Model(model, robot_name="UR10", plt_model=True)

# Define Task-Space Waypoints [x, y, z, roll, pitch, yaw] in degrees
waypoints = [
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 1
    [0.20, -0.10, 0.07, 180, 0, -30],  # Target 2
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 3
    [0.20,  0.10, 0.15, 180, 0, -30],  # Target 4
    [0.20,  0.10, 0.07, 180, 0, -90],  # Target 5
    [0.20,  0.10, 0.15, 180, 0, -30],  # Target 6
    [0.20, -0.10, 0.15, 180, 0, -90],  # Target 1
]

# Configure trajectory parameters
robot.model.set_eular_in_deg(True)
robot.traj.scale_waypoint_vel = 0.0
robot.traj.traj_type("qu")

# Generate Task-Space Trajectory
trajectory = robot.traj.create_trajectory(
    waypoints,
    traj_method="ts",
    n_samples=20
)

# Display waypoint velocities
print("Waypoint velocities:\n", robot.traj.get_waypoint_velocities())

# Visualize Trajectory
robot.mviz.scale_viz(0.3, 0.05)
robot.mviz.show_target(waypoints, axis_length=0.04, sc=True)
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_via_points=False,
    show_joint_label=True,
    repeat=True
)

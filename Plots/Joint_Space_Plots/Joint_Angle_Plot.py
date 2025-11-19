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

# Initialize Robot
model = DHModel.get_model('Puma561')
robot = Init_Model(model, robot_name='Puma561', plt_model=True)

# Define Waypoints [x, y, z, roll, pitch, yaw]
waypoints = [
    [0.2,  0.1, 0.3,  180, -90, 10],
    [0.15, 0.01, 0.1,  180, -90, 10],
    [0.2, -0.1, 0.3,  180, -90, 10],
    [0.26, 0.0, 0.17,  180, -90, 10],
    [0.2,  0.1, 0.3,  180, -90, 10],
]

# Trajectory Settings
robot.model.set_eular_in_deg(True)
robot.traj.set_traj_time(5.0)
robot.traj.scale_waypoint_vel = 0.0
robot.traj.traj_type('qu')  # Quintic trajectory

# Create Joint-Space Trajectory
trajectory = robot.traj.create_trajectory(
    waypoints, traj_method='js', n_samples=20
)

# Print waypoint velocities
print("\nWaypoint Velocities:\n", robot.traj.get_waypoint_velocities())

# Plot Joint Coordinates
robot.plotter.plot_traj(plot_type='jc', selected_plot='all')  # 'jc' = joint coordinates

# Visualize Robot Motion
robot.mviz.scale_viz(0.25, 0.04)
robot.mviz.show_target(waypoints, 0.0, sc=True)
robot.mviz.plot(
    trajectory,
    show_path=True,
    show_via_points=True,
    show_joint_label=True,
    repeat=True
)

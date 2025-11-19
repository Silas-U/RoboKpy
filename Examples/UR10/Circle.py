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


# Initialize UR10 Robot
model = DHModel.get_model("UR10")
robot = Init_Model(
    model,
    robot_name="UR10",
    plt_model=True
)

# Define Circle Trajectory in Task Space
# Format: [x, y, z, roll, pitch, yaw]
center = [0.3, 0.1, 0.1, 3.14159265, 0.0, -0.52359878]  # Center of the circle
radius = -0.07  # Negative radius rotates circle in opposite direction
circle_waypoints = robot.traj.create_circle_traj(radius=radius, cent=center)

# Convert Task-Space Waypoints to Joint Angles
joint_trajectory = robot.traj.wayp_to_joint_angle(circle_waypoints)

# Visualize Trajectory
robot.mviz.scale_viz(0.35, 0.04)
robot.mviz.show_target([center], axis_length=0.02, sc=True)
robot.mviz.plot(
    joint_trajectory,
    show_path=True,
    show_via_points=True,
    repeat=True
)

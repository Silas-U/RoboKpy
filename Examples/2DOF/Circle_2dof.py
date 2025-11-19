"""
2-DOF Circular Trajectory Demonstration
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from Robokpy import Init_Model
from Model import DHModel


def main():

    # Initialize Robot
    model_data = DHModel.get_model('2dof')
    robot = Init_Model(model_data, robot_name='2dof', plt_model=True)

    # Define Circle Trajectory in Task Space
    # Format: [x, y, z, roll, pitch, yaw]
    center = [
        0.30,        # X
        0.20,        # Y
        0.20,        # Z
        0.0,         # Roll
        0.0,         # Pitch
        1.57079633   # Yaw (~ 90 degrees)
    ]

    # Generate Circular Cartesian Trajectory
    radius = 0.09
    cartesian_circle = robot.traj.create_circle_traj(radius=radius, cent=center)

    # Convert Cartesian Waypoints to Joint Angles
    joint_traj = robot.traj.wayp_to_joint_angle(
        cartesian_circle,
        xyz_mask=[1, 1, 1, 0, 0, 0]   # position only, ignore orientation
    )

    # Visualization Setup
    robot.mviz.scale_viz(0.35, 0.05)
    robot.mviz.show_target([center], 0.0, sc=True)

    # Render the Motion
    robot.mviz.plot(
        joint_traj,
        show_path=True,
        show_via_points=False,
        show_final_pose=True,
        repeat=True
    )


if __name__ == "__main__":
    main()

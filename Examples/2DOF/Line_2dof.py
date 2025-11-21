"""
2-DOF Trajectory Demonstration
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

    # Define Cartesian Waypoints (XYZ only, zero orientation)
    waypoints = [
        [0.30, 0.20, 0.20, 0.0, 0.0, 0.0],
        [0.30, 0.02, 0.20, 0.0, 0.0, 0.0],
        [0.30, 0.20, 0.20, 0.0, 0.0, 0.0],
    ]

    # Trajectory Setup
    robot.traj.traj_type('qu')        # quintic-based
    xyz_mask = [1, 1, 1, 0, 0, 0]      # position only, ignore orientation

    # Generate Cartesian Trajectory
    trajectory = robot.traj.create_trajectory(
        waypoints,
        traj_method='ts',
        n_samples=20,
        xyz_mask=xyz_mask
    )

    # Visualization Settings
    robot.mviz.scale_viz(0.35, 0.04)
    robot.mviz.show_target(waypoints, 0.0, sc=True)

    # Render Motion in Viewer
    robot.mviz.plot(
        trajectory,
        show_path=True,
        show_via_points=True,
        show_joint_label=True,
        repeat=True
    )


if __name__ == "__main__":
    main()

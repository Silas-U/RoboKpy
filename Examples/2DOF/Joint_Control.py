"""
2-DOF Joint-Space Trajectory Demo
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

    # Display Available Models
    available_models = DHModel.list_models()
    print("\nAvailable Models:")
    print(available_models)

    # Define Joint Poses (degrees)
    joint_poses = [
        [0,    0],
        [-90,  0],
        [-90, 90],
        [0,    0],
    ]

    # Generate Joint-Space Trajectory
    robot.traj.traj_type('qu')    # quintic-based
    trajectory = robot.traj.joint_control(
        joint_poses,
        n_samples=20
    )

    # Visualization Configuration
    robot.mviz.scale_viz(0.5, 0.05)

    # Render Motion in Viewer
    robot.mviz.plot(
        trajectory,
        show_path=True,
        show_via_points=True,
        repeat=True
    )


if __name__ == "__main__":
    main()

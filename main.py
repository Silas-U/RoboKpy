"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

from robokpy import Init_Model
from Model import DHModel


def main():
    # ------------------------------------------------------------
    # Loading Robot Model
    # ------------------------------------------------------------
    model_data = DHModel.get_model('Puma561')
    robot = Init_Model(model_data, robot_name='Puma561', plt_model=True)

    # ------------------------------------------------------------
    # Defining Waypoints (position + orientation in degrees)
    # ------------------------------------------------------------
    waypoints = [
        [0.20,  0.10, 0.30, 180.0, -90.0, 10.0],
        [0.15,  0.01, 0.10, 180.0, -90.0, 10.0],
        [0.20, -0.10, 0.30, 180.0, -90.0, 10.0],
        [0.26,  0.00, 0.17, 180.0, -90.0, 10.0],
        [0.20,  0.10, 0.30, 180.0, -90.0, 10.0],
    ]

    robot.model.set_eular_in_deg(True)

    # ------------------------------------------------------------
    # Trajectory Settings
    # ------------------------------------------------------------
    robot.traj.set_traj_time(5.0)
    robot.traj.scale_waypoint_vel = 0.0
    robot.traj.traj_type('qu')

    # ------------------------------------------------------------
    # Generating Trajectory
    # ------------------------------------------------------------
    trajectory = robot.traj.create_trajectory(
        waypoints,
        traj_method='ts',
        n_samples=20
    )

    print("\nWaypoint Velocities:")
    print(robot.traj.get_waypoint_velocities())

    # ------------------------------------------------------------
    # Ploting Joint Velocities
    # ------------------------------------------------------------
    robot.plotter.plot_traj(
        plot_type='vel',
        selected_plot='all'
    )

    # ------------------------------------------------------------
    # Visualization Settings
    # ------------------------------------------------------------
    robot.mviz.scale_viz(0.25, 0.04)
    robot.mviz.show_target(waypoints, 0.0, sc=True)

    # ------------------------------------------------------------
    # Rendering Motion
    # ------------------------------------------------------------
    robot.mviz.plot(
        trajectory,
        show_path=True,
        show_via_points=True,
        show_joint_label=True,
        repeat=True
    )


if __name__ == "__main__":
    main()

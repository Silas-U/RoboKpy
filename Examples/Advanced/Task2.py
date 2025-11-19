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


def get_waypoints():
    """
    Return trajectory waypoints in the format:
    [x, y, z, roll, pitch, yaw], with orientation given in degrees.
    """
    return [
        [0.30, 0.00, 0.20, 180,   10, 0],
        [0.30, 0.00, 0.20, 110,   10, 0],
        [0.30, 0.00, 0.20, -110,  10, 0],
        [0.30, 0.10, 0.20, -110,  10, 0],
        [0.30, 0.15, 0.20, -110,  10, 0],
        [0.30, 0.20, 0.20, -110,  10, 0],
        [0.30, 0.10, 0.20, -110,  10, 0],
        [0.30, 0.00, 0.20, -110,  10, 0],
        [0.30, 0.00, 0.17, -110,  10, 0],
        [0.30, 0.00, 0.14, -110,  10, 0],
        [0.30, 0.00, 0.10, -110,  10, 0],
        [0.30, 0.10, 0.10, -110,  10, 0],
        [0.30, 0.20, 0.10, -110,  10, 0],
        [0.30, 0.10, 0.10, -110,  10, 0],
        [0.30, 0.00, 0.10, -110,  10, 0],
        [0.30, 0.00, 0.20, -110,  10, 0],
        [0.30, 0.00, 0.20, 180,   10, 0],
    ]


def main():
    """Execute a task-space trajectory on the Puma561 robot model."""
    
    # Initialize robot
    robot_model = DHModel.get_model("Puma561")
    robot = Init_Model(
        robot_model,
        robot_name="Puma561",
        twist_in_rads=False,
        plt_model=True
    )

    waypoints = get_waypoints()

    # Trajectory configuration
    robot.model.set_eular_in_deg(True)
    robot.traj.scale_waypoint_vel = 0.0
    robot.traj.traj_type("qu")

    trajectory = robot.traj.create_trajectory(
        waypoints,
        traj_method="ts",
        n_samples=7
    )

    print("\nWaypoint Velocities:")
    print(robot.traj.get_waypoint_velocities())

    # Visualization
    robot.mviz.scale_viz(0.25, 0.015)
    robot.mviz.show_target(waypoints, axis_length=0.04, sc=True)
    robot.mviz.plot(
        trajectory,
        show_path=True,
        show_via_points=False,
        show_joint_label=True,
        repeat=True
    )


if __name__ == "__main__":
    main()

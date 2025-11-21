# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import matplotlib.lines as mlines
from matplotlib.animation import FuncAnimation

class VizModel:

    def __init__(self, model, fk):
        # Robot instance from Init_Model@robokpy
        self.robot_instance = model
        self.fk = fk
        # Visualization Parameters
        self.n_points = 100
        self.scale = 0.3
        self.__scale = 0
        self.background_color = 'w'
        self.identity_rotation = np.eye(3)
        self.end_effector_positions = []
        self.arm_limits = []
        self.pt = np.zeros((self.n_points, 3))
        self.axis_len = 0
        self.restart = False  # Restart flag

        # Set up 3D plot
        plt.ion()
        self.fig = plt.figure(figsize=(6, 6), facecolor=self.background_color)
        self.fig.canvas.mpl_connect('close_event', self.handle_close)
        self.ax = self.fig.add_subplot(111, projection='3d', facecolor=self.background_color)
        self.ax.set_xlabel('X [m]', color='#101a')
        self.ax.set_ylabel('Y [m]', color='#101a')
        self.ax.set_zlabel('Z [m]', color='#101a')
        self.ax.set_xlim([-self.scale, self.scale])
        self.ax.set_ylim([-self.scale, self.scale])
        self.ax.set_zlim([-self.scale, self.scale])
        self.ax.set_title(f"RoboKpy Model: {self.robot_instance.get_robot_name()}")

        self.ax.tick_params(axis='x', colors='#101a')
        self.ax.tick_params(axis='y', colors='#101a')
        self.ax.tick_params(axis='z', colors='#101a')

        # Initialize plot elements
        self.via_points, = self.ax.plot([], [], [], '+', color="#0F4C5F")
        self.arm_line_final, = self.ax.plot([], [], [], '--', color="#094E63", lw=3)
        self.arm_line, = self.ax.plot([], [], [], '-+', color='black', lw=16)
        self.jnt, = self.ax.plot([], [], [], '-+', color='#eff5', lw=15)
        self.end_effector_line, = self.ax.plot([], [], [], '-', color="#DB03C9", label='Trajectory')

        self.quivers = []
        self.sctr = []

    def handle_close(self, evnt):
        print(f"$ --- Plot Closed --- : >> {self.robot_instance.get_robot_name()} Robot")
        self.stop_loop = True

    @staticmethod
    def rotation_matrix(axis, angle):
        return R.from_euler(axis, angle, degrees=True).as_matrix()

    @staticmethod
    def plot_coordinate_frame(ax, origin, rotation_matrix, axis_length=0):
        x_axis = origin + rotation_matrix @ np.array([axis_length, 0, 0])
        y_axis = origin + rotation_matrix @ np.array([0, axis_length, 0])
        z_axis = origin + rotation_matrix @ np.array([0, 0, axis_length])
        x_quiver = ax.quiver(*origin, *(x_axis - origin), color="#fc0a0e", arrow_length_ratio=0.0, linewidths=2.5)
        y_quiver = ax.quiver(*origin, *(y_axis - origin), color="#19eb15", arrow_length_ratio=0.0, linewidths=2.5)
        z_quiver = ax.quiver(*origin, *(z_axis - origin), color="#1122d6", arrow_length_ratio=0.0, linewidths=2.5)
        return [x_quiver, y_quiver, z_quiver]

    def show_target(self, tar, axis_length=0, sc=False):
        if type(tar) not in [np.ndarray, list]:
            raise TypeError(f"Expected a list of cartesian targets but got {type(tar)}")
        if type(axis_length) not in [int, float]:
            raise TypeError("axis_length must be of type integer or float")
        for i in range(len(tar)):
            self.plot_coordinate_frame(self.ax, tar[i][:3], self.identity_rotation, axis_length=axis_length)
            if sc:
                self.ax.scatter(tar[i][:3][0], tar[i][:3][1], tar[i][:3][2], c=tar[i][:3][2], cmap='viridis', marker='o')

   # Initialize function to clear the plot
    def init(self):
        self.jnt.set_data([], [])
        self.jnt.set_3d_properties([])
        self.arm_line.set_data([], [])
        self.arm_line.set_3d_properties([])

        self.arm_line_final.set_data([], [])
        self.arm_line_final.set_3d_properties([])

        self.via_points.set_data([], [])
        self.via_points.set_3d_properties([])

        self.end_effector_line.set_data([], [])
        self.end_effector_line.set_3d_properties([])


    def cumulative_rotation(self, joint_angles, points, rotation_axes, robot_instance, twist_angle):
        current_position = np.array([0, 0, 0, 1])  # Homogeneous coordinates
        current_rotation = np.eye(3)

        twist_in_deg = []
        if robot_instance.l_twist_in_rads():
            twist_in_deg = np.rad2deg(twist_angle)
        elif not robot_instance.l_twist_in_rads():
            twist_in_deg = twist_angle

        # Plot the reference (global) frame
        self.quivers += self.plot_coordinate_frame(self.ax, current_position[:3], self.identity_rotation)

        # Loop through each link and apply cumulative transformations
        for i, (theta, point, axis, twist) in enumerate(zip(joint_angles, points, rotation_axes, twist_in_deg)):

            twist_r = self.rotation_matrix('x', twist)
            rotation_matrix_i = self.rotation_matrix(axis, theta)
            current_rotation = np.dot(current_rotation, rotation_matrix_i) @ -twist_r
            link_end = current_position[-1] + current_rotation @ np.array(point)
            self.quivers += self.plot_coordinate_frame(self.ax,  point, (current_rotation @ twist_r), axis_length=self.axis_len)
            current_position = np.hstack((link_end, [1]))  # Homogeneous coordinates

   
    def plot(self, trajectory, show_path=False, show_joint_label=False, show_frame_label=False, show_via_points=False, show_final_pose=False, repeat=False):
        
        if type(trajectory) not in [np.ndarray, list]:
            raise TypeError(f"Expected a list of cartesian waypoints but got {type(trajectory)}")
        
        self.init()
        self.end_effector_positions = []
        self.play = True
        self.restart = False

        # Keep a reference to the animation to avoid garbage collection
        if hasattr(self, "anim") and self.anim:
            del self.anim  # clear any old animation

        # Legend setup
        x_line = mlines.Line2D([], [], color="#fc0a0e", label='X-axis')
        y_line = mlines.Line2D([], [], color="#19eb15", label='Y-axis')
        z_line = mlines.Line2D([], [], color="#1122d6", label='Z-axis')
        self.ax.legend(handles=[x_line, y_line, z_line], loc='upper right')

        nj = self.robot_instance.get_num_of_joints()
        joint_type = self.robot_instance.get_joint_type()
        new = []

        for n in range(len(trajectory)):
            for x in range(len(trajectory[n])):
                for y in range(nj):
                    if joint_type[y] == "r":
                        new.append(np.degrees(trajectory[n][x][y]))
                    elif joint_type[y] == "p":
                        new.append(trajectory[n][x][y])

        new_traj = np.split(np.array(new), len(new) / nj)
        self.traj_data = new_traj

        jnt_conf = self.robot_instance.get_jnt_configs()
        self.limit_p = []
        for x in range(len(jnt_conf)):
            self.fk.compute(jnt_conf[x], rads=True)
            transform_length = self.fk.transform_length()
            jnts = [self.fk.get_j_origin(i + 1) for i in range(transform_length)]
            pnts = [[0, 0, 0]]
            for i in jnts:
                pnts.append(i)
            self.limit_p = np.array(pnts)

        self.joint_labels = []
        self.frame_labels = []

        def update(frame):
            q = self.traj_data[int(frame)]
            self.fk.compute(q, rads=False)

            # Remove old quivers
            for quiver in self.quivers:
                quiver.remove()
            self.quivers = []

            dhparams = self.robot_instance.get_dh_params()
            transform_length = self.fk.transform_length()
            joints = [self.fk.get_j_origin(i + 1) for i in range(transform_length)]
            points = [[0, 0, 0]]
            for i in joints:
                points.append(i)
            p = np.array(points)

            if self.robot_instance.get_num_of_joints() == 2:
                indexes = [i for i in range(0, transform_length, 2)]
            else:
                indexes = [i for i in range(0, transform_length - 2, 2)]
                indexes.append(-1)

            link_lengths = [p[i] for i in indexes]
            rotation_axes = ['z'] * transform_length
            twist = [dh[3] for dh in dhparams]

            self.arm_line.set_data(p[:, 0], p[:, 1])
            self.arm_line.set_3d_properties(p[:, 2])
            self.jnt.set_data(p[:, 0], p[:, 1])
            self.jnt.set_3d_properties(p[:, 2])

            if show_frame_label:
                frame_names = ["frame0", "frame1", "frame2", "frame3", "frame4", "frame5", "frame6", "frame7"]
                for lbl in self.frame_labels:
                    lbl.remove()
                self.frame_labels = []

                default_frame_names = [f"frame_{i}" for i in range(0, len(link_lengths))]
                fnames = frame_names if frame_names else default_frame_names

                for i, (xj, yj, zj) in enumerate(link_lengths):
                    fname = fnames[i] if i < len(fnames) else default_frame_names[i]
                    lbl = self.ax.text(
                        xj, yj, zj,
                        fname,
                        fontsize=9,
                        color="#1c1c3c",
                        fontweight="bold",
                        ha="left",
                    )
                    self.frame_labels.append(lbl)

            if show_joint_label:
                joint_names = self.robot_instance.get_joint_names()

                for lbl in self.joint_labels:
                    lbl.remove()
                self.joint_labels = []

                default_names = [f"J{i}" for i in range(0, len(link_lengths))]
                names = joint_names if joint_names else default_names

                for i, (xj, yj, zj) in enumerate(link_lengths):
                    name = names[i] if i < len(names) else default_names[i]
                    lbl = self.ax.text(
                        xj+0.01, yj+0.01, zj+0.04,
                        name,
                        fontsize=9,
                        color="#F72626",
                        fontweight="bold",
                        ha="right",
                    )
                    self.joint_labels.append(lbl)

            self.end_effector_positions.append(p[-1, :])
            p_end_effector = np.array(self.end_effector_positions)

            if show_path:
                self.end_effector_line.set_data(p_end_effector[:, 0], p_end_effector[:, 1])
                self.end_effector_line.set_3d_properties(p_end_effector[:, 2])

            if show_final_pose:
                self.arm_line_final.set_data(self.limit_p[:, 0], self.limit_p[:, 1])
                self.arm_line_final.set_3d_properties(self.limit_p[:, 2])

            if show_via_points:
                self.via_points.set_data(p_end_effector[:, 0], p_end_effector[:, 1])
                self.via_points.set_3d_properties(p_end_effector[:, 2])


            self.cumulative_rotation(q, link_lengths, rotation_axes, self.robot_instance, twist)
            return self.arm_line, self.jnt, self.end_effector_line
        
        self.anim = FuncAnimation(
            self.fig,
            update,
            frames=len(new_traj),
            interval=100,
            blit=False,
            repeat=repeat
        )

        plt.tight_layout()
        plt.show(block=True)

    def scale_viz(self, scale, fram_axis_len=0):
        if type(scale) not in [int, float]:
            raise TypeError("scale must be of type integer or float")
        if type(fram_axis_len) not in [int, float]:
            raise TypeError("fram_axis_len be of type integer or float")
        
        self.__scale = scale
        self.axis_len = fram_axis_len
        self.ax.set_xlim([-scale, scale])
        self.ax.set_ylim([-scale, scale])
        self.ax.set_zlim([-scale, scale])

    def show_dh_model(self, joints_v=None, show_joint_label=False, show_frame_label=False):
        if type(joints_v) not in [np.ndarray, list]:
            raise TypeError(f"Expected a list of cartesian waypoints but got {type(joints_v)}")
    
        n_joints = self.robot_instance.get_num_of_joints()

        if joints_v is None:
            joints_vals = np.zeros(n_joints)
        else:
            if len(joints_v) != n_joints:
                raise ValueError(
                    f"Expected {n_joints} but received {len(joints_v)} joint values for a {n_joints}-axis robot"
                )
            for item in joints_v:
                if not isinstance(item, (int, float)):
                    raise TypeError("Input must be of type integer or float")
            joints_vals = np.array(joints_v, dtype=float)

        joint_state = []
        for i in range(len(joints_vals)):
            if self.robot_instance.get_dh_params()[i][1] == "r":
                joint_state.append(np.deg2rad(joints_vals[i]) if self.robot_instance.joint_states_deg else joints_vals[i])
            else:  # prismatic
                joint_state.append(joints_vals[i])
        
        self.plot([[joint_state]], show_frame_label=show_frame_label, show_joint_label=show_joint_label, show_path=True)
 
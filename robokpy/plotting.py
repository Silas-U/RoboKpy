"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

import matplotlib.pyplot as plt
import numpy as np

class Plotter:
    def __init__(self, model, fk,  trajectory):
        self.model = model
        self.fk = fk
        self.trajectory = trajectory

    def __check_attr(self, *attrs):
        for attr in attrs:
            if not hasattr(self.trajectory, attr):
                print(f"[Warning] Missing trajectory data: '{attr}' — skipping this plot.")
                return False
            value = getattr(self.trajectory, attr)
            if value is None or (isinstance(value, (list, np.ndarray)) and len(value) == 0):
                print(f"[Warning] '{attr}' data is not available for this traj type — skipping this plot.")
                return False
        return True

    def plot_jointspace_traj(self, plot_type='xyz', selected_plot="all", projection="2d"):     
        pose  =  []
        try:
            for p in self.trajectory.trajectory:
                for j in range(len(p)):
                    self.fk.compute(p[j], rads=True)
                    pose.append(self.fk.get_target())
        except Exception as e:
            print(f"[Error] Failed to compute forward kinematics for joint-space trajectory: {e}")
            return

        pos_x= [pose[i][0] for i in range(len(pose))]
        pos_y = [pose[i][1] for i in range(len(pose))]
        pos_z = [pose[i][2] for i in range(len(pose))]

        position = [pos_x, pos_y, pos_z]

        roll = [pose[i][5] for i in range(len(pose))]
        pitch = [pose[i][4] for i in range(len(pose))]
        yaw = [pose[i][3] for i in range(len(pose))]

        orientation = [roll, pitch, yaw]

        num_of_joints = self.model.get_num_of_joints()

        # Safely handle each plot type
        try:    
            if plot_type == 'jc':
                if not self.__check_attr('jq', 't_fine'):
                    return
                # === Plot Position ===
                plt.figure(figsize=(6, 6))
                for j in range(num_of_joints):
                    plt.plot(self.trajectory.t_fine, self.trajectory.jq[j], label=f'Joint {j+1}')
                plt.xlabel("Time (s)")
                plt.ylabel("Joint coordinates (rads)")
                plt.legend()
                plt.grid(True)
                plt.tight_layout()
                plt.show()
            elif plot_type == 'vel':
                if not self.__check_attr('jq_vel', 't_fine'):
                    return
                # === Plot Velocity ===
                plt.figure(figsize=(6, 6))
                for j in range(num_of_joints):
                    plt.plot(self.trajectory.t_fine, self.trajectory.jq_vel[j], label=f'Joint {j+1}')
                plt.xlabel("Time (s)")
                plt.ylabel("Velocity (m/s)")
                plt.legend()
                plt.grid(True)
                plt.tight_layout()
                plt.show()
            elif plot_type == 'acc':
                if not self.__check_attr('jq_acc', 't_fine'):
                    return
                # === Plot Acceleration ===
                plt.figure(figsize=(6, 6))
                for j in range(num_of_joints):
                    plt.plot(self.trajectory.t_fine, self.trajectory.jq_acc[j], label=f'Joint {j+1}')
                plt.xlabel("Time (s)")
                plt.ylabel("Acceleration (m/s²)")
                plt.legend()
                plt.grid(True)
                plt.tight_layout()
                plt.show()
            elif plot_type == 'jer':
                if not self.__check_attr('jq_jerk', 't_fine'):
                    return
                # === Plot Jerk ===
                plt.figure(figsize=(6, 6))
                for j in range(num_of_joints):
                    plt.plot(self.trajectory.t_fine, self.trajectory.jq_jerk[j], label=f'Joint {j+1}')
                plt.xlabel("Time (s)")
                plt.ylabel("Jerk")
                plt.legend()
                plt.grid(True)
                plt.tight_layout()
                plt.show()
            elif plot_type == 'xyz':
                quantities = [
                    (np.array(position[0]), np.array(position[1]), np.array(position[2]), 'Position (m)')
                ]
                plt.figure(figsize=(6, 6))
                for i, (qx, qy, qz, title) in enumerate(quantities):
                    if selected_plot == "x":
                        plt.plot(self.trajectory.t_fine, qx, label='x', color="#1E7BF5")
                    elif selected_plot == "y":
                        plt.plot(self.trajectory.t_fine, qy, label='y', color="#E68507")
                    elif selected_plot == "z":
                        plt.plot(self.trajectory.t_fine, qz, label='z', color="#FDDC21")
                    elif selected_plot == "all":
                        plt.plot(self.trajectory.t_fine, qx, label='x', color="#1E7BF5")
                        plt.plot(self.trajectory.t_fine, qy, label='y', color="#E68507")
                        plt.plot(self.trajectory.t_fine, qz, label='z', color="#FDDC21")
                    else:
                        raise ValueError(f"Unsupported plot type {selected_plot}: supported -> x:y:z:all")
                
                plt.ylabel(title)
                plt.legend()
                plt.grid(True)

                plt.xlabel("Time (s)")
                plt.tight_layout()
                plt.show()
            
            elif plot_type == 'rpy':
                quantities = [
                    (orientation[0], orientation[1], orientation[2], 'RPY (rads)')
                ]
                plt.figure(figsize=(6, 6))
                for i, (rx, ry, rz, title) in enumerate(quantities):
                    if selected_plot == "r":
                        plt.plot(self.trajectory.t_fine, rx, label='Roll', color="#1E7BF5")
                    elif selected_plot == "p":
                        plt.plot(self.trajectory.t_fine, ry, label='Pitch', color="#E68507")
                    elif selected_plot == "y":
                        plt.plot(self.trajectory.t_fine, rz, label='Yaw', color="#FDDC21")
                    elif selected_plot == "all":
                        plt.plot(self.trajectory.t_fine, rx, label='Roll', color="#1E7BF5")
                        plt.plot(self.trajectory.t_fine, ry, label='Pitch', color="#E68507")
                        plt.plot(self.trajectory.t_fine, rz, label='Yaw', color="#FDDC21")
                    else:
                        raise ValueError(f"Unsupported plot type {selected_plot}: supported -> r:p:y:all")
                        
                plt.ylabel(title)
                plt.legend()
                plt.grid(True)

                plt.xlabel("Time (s)")
                plt.tight_layout()
                plt.show()
            
            elif plot_type == 'traj':

                self.fig = plt.figure(figsize=(6, 6))

                if projection == '2d':
                    ax = self.fig.add_subplot(111)
                    ax.set_xlabel('X (m)')
                    ax.set_ylabel('Y (m)')

                    # Plot trajectory
                    ax.plot(position[0], position[1], '-', color="#DB03C9", label='Trajectory')

                    ax.set_aspect('equal', adjustable='datalim')

                    start_x, start_y = position[0][0], position[1][0]
                    end_x, end_y = position[0][-1], position[1][-1]
                    ax.scatter(start_x, start_y, color='#6BDB03', s=60, label='Start', zorder=5)
                    ax.scatter(end_x, end_y, color='#0385DB', s=60, label='End', zorder=5)

                    ax.text(start_x, start_y + 0.0005, "Start", fontsize=9, fontweight='bold')
                    ax.text(end_x, end_y + 0.0005, "End", fontsize=9, fontweight='bold')

                elif projection == '3d':
                    ax = self.fig.add_subplot(111, projection='3d')
                    ax.set_xlabel('X (m)')
                    ax.set_ylabel('Y (m)')
                    ax.set_zlabel('Z (m)')
                    ax.plot(position[0], position[1], position[2], '-', color="#2c6feb", label='Trajectory')

                    start_x, start_y, start_z = position[0][0], position[1][0], position[2][0]
                    end_x, end_y, end_z = position[0][-1], position[1][-1], position[2][-1]
                    ax.scatter(start_x, start_y, start_z, color="#6BDB03", s=60, label='Start', depthshade=True)
                    ax.scatter(end_x, end_y, end_z, color="#0385DB", s=60, label='End', depthshade=True)

                    # 3D text labels
                    ax.text(start_x, start_y + 0.0005, start_z + 0.0005, "Start", fontsize=9, fontweight='bold')
                    ax.text(end_x, end_y + 0.0005, end_z + 0.0005, "End", fontsize=9, fontweight='bold')

                    x_limits = [np.min(position[0]), np.max(position[0])]
                    y_limits = [np.min(position[1]), np.max(position[1])]
                    z_limits = [np.min(position[2]), np.max(position[2])]
                    max_range = np.ptp([*x_limits, *y_limits, *z_limits]) / 2.0
                    mid_x = np.mean(x_limits)
                    mid_y = np.mean(y_limits)
                    mid_z = np.mean(z_limits)
                    ax.set_xlim(mid_x - max_range, mid_x + max_range)
                    ax.set_ylim(mid_y - max_range, mid_y + max_range)
                    ax.set_zlim(mid_z - max_range, mid_z + max_range)

                else:
                    raise ValueError(f"Unsupported projection {projection}: supported -> 2d:3d")

                plt.legend()
                plt.grid(True)
                plt.tight_layout()
                plt.show()

            else:
                raise ValueError(f"Unsupported plot type {plot_type}: supported -> jc:xyz:rpy")
        except Exception as e:
            print(f"[Error] Failed to plot {plot_type} trajectory: {e}")


    def plot_taskspace_traj(self, plot_type='xyz', selected_plot="all", projection="2d"):
        # Safely handle each plot type
            num_of_joints = self.model.get_num_of_joints()
            try:  
                tr = np.array(np.transpose(self.trajectory.trajectory))
                if plot_type == 'jc':
                        if not self.__check_attr('trajectory', 't_fine'):
                            return
                        # === Plot Position ===
                        plt.figure(figsize=(6, 6))
                        for j in range(num_of_joints):
                            plt.plot(self.trajectory.t_fine, tr[j], label=f'Joint {j+1}')
                        plt.xlabel("Time (s)")
                        plt.ylabel("Joint coordinates (rads)")
                        plt.legend()
                        plt.grid(True)
                        plt.tight_layout()
                        plt.show()

                elif plot_type == 'vel':
                    quantities = [
                        (self.trajectory.vel_x, self.trajectory.vel_y, self.trajectory.vel_z, 'Velocity (m/s)')
                    ]
                    if not self.__check_attr('vel_x', 'vel_y', 'vel_z', 't_fine'):
                            return
                    plt.figure(figsize=(6, 6))
                    for i, (vx, vy, vz, title) in enumerate(quantities):
                        if selected_plot == "x":
                            plt.plot(self.trajectory.t_fine, vx, label='x', color='#1E7BF5')
                        elif selected_plot == "y":
                            plt.plot(self.trajectory.t_fine, vy, label='y', color='#E68507')
                        elif selected_plot == "z":
                            plt.plot(self.trajectory.t_fine, vz, label='z', color='#FDDC21')
                        elif selected_plot == "all":
                            plt.plot(self.trajectory.t_fine, vx, label='x', color='#1E7BF5')
                            plt.plot(self.trajectory.t_fine, vy, label='y', color='#E68507')
                            plt.plot(self.trajectory.t_fine, vz, label='z', color='#FDDC21')
                        else:
                            raise ValueError(f"Unsupported plot type {selected_plot}: supported -> x:y:z:all")
                    
                    plt.ylabel(title)
                    plt.legend()
                    plt.grid(True)

                    plt.xlabel("Time (s)")
                    plt.tight_layout()
                    plt.show()
                
                elif plot_type == 'jer':
                    quantities = [
                        (self.trajectory.jerk_x, self.trajectory.jerk_y, self.trajectory.jerk_z, 'Jerk')
                    ]
                    if not self.__check_attr('jerk_x', 'jerk_y', 'jerk_z', 't_fine'):
                            return
                    plt.figure(figsize=(6, 6))
                    for i, (jx, jy, jz, title) in enumerate(quantities):
                        if selected_plot == "x":
                            plt.plot(self.trajectory.t_fine, jx, label='x', color='#1E7BF5')
                        elif selected_plot == "y":
                            plt.plot(self.trajectory.t_fine, jy, label='y', color='#E68507')
                        elif selected_plot == "z":
                            plt.plot(self.trajectory.t_fine, jz, label='z', color='#FDDC21')
                        elif selected_plot == "all":
                            plt.plot(self.trajectory.t_fine, jx, label='x', color='#1E7BF5')
                            plt.plot(self.trajectory.t_fine, jy, label='y', color='#E68507')
                            plt.plot(self.trajectory.t_fine, jz, label='z', color='#FDDC21')
                        else:
                            raise ValueError(f"Unsupported plot type {selected_plot}: supported -> x:y:z:all")
                    
                    plt.ylabel(title)
                    plt.legend()
                    plt.grid(True)

                    plt.xlabel("Time (s)")
                    plt.tight_layout()
                    plt.show()

                elif plot_type == 'acc':
                    quantities = [
                        (self.trajectory.acc_x, self.trajectory.acc_y, self.trajectory.acc_z, 'Acceleration (m/s²)')
                    ]
                    if not self.__check_attr('acc_x', 'acc_y', 'acc_z', 't_fine'):
                            return
                    plt.figure(figsize=(6, 6))
                    for i, (acx, acy, acz, title) in enumerate(quantities):
                        if selected_plot == "x":
                            plt.plot(self.trajectory.t_fine, acx, label='x', color='#1E7BF5')
                        elif selected_plot == "y":
                            plt.plot(self.trajectory.t_fine, acy, label='y', color='#E68507')
                        elif selected_plot == "z":
                            plt.plot(self.trajectory.t_fine, acz, label='z', color='#FDDC21')
                        elif selected_plot == "all":
                            plt.plot(self.trajectory.t_fine, acx, label='x', color='#1E7BF5')
                            plt.plot(self.trajectory.t_fine, acy, label='y', color='#E68507')
                            plt.plot(self.trajectory.t_fine, acz, label='z', color='#FDDC21')
                        else:
                            raise ValueError(f"Unsupported plot type {selected_plot}: supported -> x:y:z:all")
                    
                    plt.ylabel(title)
                    plt.legend()
                    plt.grid(True)

                    plt.xlabel("Time (s)")
                    plt.tight_layout()
                    plt.show()
                    
                elif plot_type == 'xyz':
                    quantities = [
                        (self.trajectory.pos_x, self.trajectory.pos_y, self.trajectory.pos_z, 'Position (m)')
                    ]
                    if not self.__check_attr('pos_x', 'pos_y', 'pos_z', 't_fine'):
                            return
                    plt.figure(figsize=(6, 6))
                    for i, (qx, qy, qz, title) in enumerate(quantities):
                        if selected_plot == 'x':
                            plt.plot(self.trajectory.t_fine, qx, label='x', color='#1E7BF5')
                        elif selected_plot == 'y':
                            plt.plot(self.trajectory.t_fine, qy, label='y', color='#E68507')
                        elif selected_plot == 'z':
                            plt.plot(self.trajectory.t_fine, qz, label='z', color='#FDDC21')
                        elif selected_plot == 'all':
                            plt.plot(self.trajectory.t_fine, qx, label='x', color='#1E7BF5')
                            plt.plot(self.trajectory.t_fine, qy, label='y', color='#E68507')
                            plt.plot(self.trajectory.t_fine, qz, label='z', color='#FDDC21')
                        else:
                            raise ValueError(f"Unsupported type {selected_plot}: supported -> x:y:z:all")
                            
                    plt.ylabel(title)
                    plt.xlabel("Time (s)")
                    plt.legend()
                    plt.grid(True)
                    plt.tight_layout()
                    plt.show()

                elif plot_type == 'rpy':
                    quantities = [
                        (self.trajectory.roll, self.trajectory.pitch, self.trajectory.yaw, 'RPY (rads)')
                    ]
                    if not self.__check_attr('roll', 'pitch', 'yaw', 't_fine'):
                            return
                    plt.figure(figsize=(6, 6))
                    for i, (rx, ry, rz, title) in enumerate(quantities):
                        if selected_plot == "r":
                            plt.plot(self.trajectory.t_fine, rx, label='Roll', color='#1E7BF5')
                        elif selected_plot == "p":
                            plt.plot(self.trajectory.t_fine, ry, label='Pitch', color='#E68507')
                        elif selected_plot == "y":
                            plt.plot(self.trajectory.t_fine, rz, label='Yaw', color='#FDDC21')
                        elif selected_plot == "all":
                            plt.plot(self.trajectory.t_fine, rx, label='Roll', color="#1E7BF5")
                            plt.plot(self.trajectory.t_fine, ry, label='Pitch', color="#E68507")
                            plt.plot(self.trajectory.t_fine, rz, label='Yaw', color="#FDDC21")
                        else:
                            raise ValueError(f"Unsupported plot type {selected_plot}: supported -> r:p:y:all")
                            
                        plt.ylabel(title)
                        plt.legend()
                        plt.grid(True)

                        plt.xlabel("Time (s)")
                        plt.tight_layout()
                        plt.show()

                elif plot_type == 'traj':

                    self.fig = plt.figure(figsize=(6, 6))

                    if projection == '2d':
                        ax = self.fig.add_subplot(111)
                        ax.set_xlabel('X (m)')
                        ax.set_ylabel('Y (m)')

                        # Plot trajectory
                        ax.plot(self.trajectory.pos_x, self.trajectory.pos_y, '-', color="#DB03C9", label='Trajectory')

                        ax.set_aspect('equal', adjustable='datalim')

                        start_x, start_y, start_z = self.trajectory.pos_x[0], self.trajectory.pos_y[0], self.trajectory.pos_z[0]
                        end_x, end_y, end_z = self.trajectory.pos_x[-1], self.trajectory.pos_y[-1], self.trajectory.pos_z[-1]

                        ax.scatter(start_x, start_y, color='#6BDB03', s=60, label='Start', zorder=5)
                        ax.scatter(end_x, end_y, color='#0385DB', s=60, label='End', zorder=5)

                        ax.text(start_x, start_y + 0.0005, "Start", fontsize=9, fontweight='bold')
                        ax.text(end_x, end_y + 0.0005, "End", fontsize=9, fontweight='bold')

                    elif projection == '3d':
                        ax = self.fig.add_subplot(111, projection='3d')
                        ax.set_xlabel('X (m)')
                        ax.set_ylabel('Y (m)')
                        ax.set_zlabel('Z (m)')
                        ax.plot(self.trajectory.pos_x, self.trajectory.pos_y, self.trajectory.pos_z, '-', color="#DB03C9", label='Trajectory')
                
                        # Start and end points
                        start_x, start_y, start_z = self.trajectory.pos_x[0], self.trajectory.pos_y[0], self.trajectory.pos_z[0]
                        end_x, end_y, end_z = self.trajectory.pos_x[-1], self.trajectory.pos_y[-1], self.trajectory.pos_z[-1]

                        ax.scatter(start_x, start_y, start_z, color="#6BDB03", s=60, label='Start', depthshade=True)
                        ax.scatter(end_x, end_y, end_z, color="#0385DB", s=60, label='End', depthshade=True)

                        # 3D text labels
                        ax.text(start_x, start_y + 0.0005, start_z + 0.0005, "Start", fontsize=9, fontweight='bold')
                        ax.text(end_x, end_y + 0.0005, end_z + 0.0005, "End", fontsize=9, fontweight='bold')

                        x_limits = [np.min(self.trajectory.pos_x), np.max(self.trajectory.pos_x)]
                        y_limits = [np.min(self.trajectory.pos_y), np.max(self.trajectory.pos_y)]
                        z_limits = [np.min(self.trajectory.pos_z), np.max(self.trajectory.pos_z)]
                        max_range = np.ptp([*x_limits, *y_limits, *z_limits]) / 2.0
                        mid_x = np.mean(x_limits)
                        mid_y = np.mean(y_limits)
                        mid_z = np.mean(z_limits)
                        ax.set_xlim(mid_x - max_range, mid_x + max_range)
                        ax.set_ylim(mid_y - max_range, mid_y + max_range)
                        ax.set_zlim(mid_z - max_range, mid_z + max_range)

                    else:
                        raise ValueError(f"Unsupported projection {projection}: supported -> 2d:3d")

                    plt.legend()
                    plt.grid(True)
                    plt.tight_layout()
                    plt.show()
                else:
                    raise ValueError(f"Unsupported plot type {plot_type}: supported -> jc:xyz:rpy")
            except Exception as e:
                print(f"[Error] Failed to plot {plot_type} trajectory: {e}")


    def plot_traj(self, plot_type='xyz', selected_plot='all', projection="2d"):
        if self.trajectory.traj_method == 'js':
            self.plot_jointspace_traj(plot_type=plot_type, selected_plot=selected_plot, projection=projection)
        elif self.trajectory.traj_method == 'ts':
            self.plot_taskspace_traj(plot_type=plot_type, selected_plot=selected_plot, projection=projection)

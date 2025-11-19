"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation as R, Slerp

class TrajectoryPlanner:
    def __init__(self, model, fk, ik, jacobian):
        self.model = model
        self.fk = fk
        self.ik = ik
        self.jacobian = jacobian
        self.pos = None
        self.jnt_configs = None
        self.scale_waypoint_vel = 0.0
        self.velocities = None

        self.t  = None
        self.q  = None
        self.qd = None
        self.qdd = None

        #plt data
        self.t_fine = None
        self.pos_x = None
        self.pos_y = None
        self.pos_z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

        #velocity
        self.vel_x = None
        self.vel_y = None
        self.vel_z = None
        self.vel_roll = None
        self.vel_pitch = None
        self.vel_yaw = None

        # acceleration
        self.acc_x = None
        self.acc_y = None
        self.acc_z = None
        self.acc_roll = None
        self.acc_pitch = None
        self.acc_yaw = None

        # jerk
        self.jerk_x = None
        self.jerk_y = None
        self.jerk_z = None
        self.jerk_roll = None
        self.jerk_pitch = None
        self.jerk_yaw = None

        self.jq = None
        self.jq_vel = None
        self.jq_acc = None
        self.jq_jerk = None

        self.t_traj = 1.0
        self.traj_method = None

    def set_traj_time(self, t_period):
        self.t_traj = t_period
    
    def get_waypoint_velocities(self):
        return self.velocities

    def wayp_to_joint_angle(self, waypoints, xyz_mask=None):
        print(f":Calculating required joint angles for trajectory...")
        try:
            joint_angles = [self.ik.solve(waypoints[i], mask=xyz_mask) for i in range(len(waypoints))]
            self.model.jnt_configs = [joint_angles]
            return [joint_angles]
        except ValueError as e:
            print(e)

    def traj_type(self, tr_type='qu'):
        """
        Set the trajectory polynomial method ('cubic' or 'quintic').
        """
        assert tr_type in ('cu', 'qu', 'spl'), "trajectory type must be 'cu (cubic)', 'qu (quintic)' or spl (spline)"
        self.tr_type = tr_type

    def cubic_segment(self, q0, q1, v0=0, v1=0, t0=0, t1=1):
        """
        Compute cubic polynomial coefficients for one trajectory segment.
        Solves for a0..a3 given boundary positions and velocities.
        """
        T = t1 - t0
        M = np.array([
            [1, t0, t0**2, t0**3],
            [0, 1,  2*t0,  3*t0**2],
            [1, t1, t1**2, t1**3],
            [0, 1,  2*t1,  3*t1**2]
        ])
        b = np.array([q0, v0, q1, v1])
        return np.linalg.solve(M, b)

    def cubic_trajectory_nd(self, waypoints, time_points, velocities=None):
        """
        Multi-DOF cubic polynomial trajectory generator.
        Returns coefficients for each dimension and segment.
        """
        waypoints = np.array(waypoints)
        m, n = waypoints.shape
        velocities = np.zeros_like(waypoints) if velocities is None else np.array(velocities)

        coeffs = [[] for _ in range(m)]
        for d in range(m):
            for i in range(n - 1):
                a = self.cubic_segment(
                    waypoints[d, i], waypoints[d, i+1],
                    velocities[d, i], velocities[d, i+1],
                    t0=time_points[i], t1=time_points[i+1]
                )
                coeffs[d].append(a)
        return coeffs

    def eval_cubic(self, coeffs, t):
        """
        Given coefficients a0..a3 for q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        Returns q, qd, qdd arrays for times t.
        """
        a0, a1, a2, a3 = coeffs
        q   = a0 + a1*t + a2*t**2 + a3*t**3
        qd  = a1 + 2*a2*t + 3*a3*t**2
        qdd = 2*a2 + 6*a3*t
        return q, qd, qdd

    def quintic_segment(self, q0, q1, v0=0, v1=0, a0=0, a1=0, t0=0, t1=1):
        """
        Compute quintic polynomial coefficients for one trajectory segment.
        """
        T = t1 - t0
        M = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [1, t1, t1**2, t1**3, t1**4, t1**5],
            [0, 1, 2*t1, 3*t1**2, 4*t1**3, 5*t1**4],
            [0, 0, 2, 6*t1, 12*t1**2, 20*t1**3]
        ])
        b = np.array([q0, v0, a0, q1, v1, a1])
        return np.linalg.solve(M, b)


    def quintic_trajectory_nd(self, waypoints, time_points, velocities=None):
        """
        Multi-DOF quintic polynomial trajectory generator.
        Returns coefficients for each dimension and segment.
        """
        waypoints = np.array(waypoints)
        m, n = waypoints.shape
        velocities = np.zeros_like(waypoints) if velocities is None else np.array(velocities)
        
        coeffs = [[] for _ in range(m)]
        for d in range(m):  # each joint
            for i in range(n - 1):  # each segment
                a = self.quintic_segment(
                    waypoints[d, i], waypoints[d, i+1],
                    velocities[d, i], velocities[d, i+1],
                    t0=time_points[i], t1=time_points[i+1]
                )
                coeffs[d].append(a)
        return coeffs
    
    def eval_quintic(self, coeffs, t):
        """
        Given coefficients a0..a5 for:
        q(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
        Returns q, qd, qdd as arrays for times t.
        """
        a0,a1,a2,a3,a4,a5 = coeffs
        q   = a0 + a1*t +   a2*t**2 +   a3*t**3 +   a4*t**4 +   a5*t**5
        qd  =      a1   + 2*a2*t     + 3*a3*t**2   + 4*a4*t**3   + 5*a5*t**4
        qdd =           2*a2        + 6*a3*t       + 12*a4*t**2    + 20*a5*t**3
        return q, qd, qdd


    def evaluate_full_trajectory(self, coeffs, time_points, n_samples=100):
        """
        Evaluate multi-segment trajectory for all dimensions.
        Returns time, q, qd, qdd arrays.
        """
        m = len(coeffs)
        # polynomial method
        tr_type = getattr(self, 'tr_type', 'qu')  # default to quintic

        t_full, q_full, qd_full, qdd_full = [], [[] for _ in range(m)], [[] for _ in range(m)], [[] for _ in range(m)]
        
        for i in range(len(time_points) - 1):
            t0, t1 = time_points[i], time_points[i+1]
            t_seg = np.linspace(t0, t1, n_samples)
            if i > 0:
                t_seg = t_seg[1:]
            t_full.extend(t_seg)

            if tr_type == 'cu':
                for d in range(m):
                    q, qd, qdd = self.eval_cubic(coeffs[d][i], t_seg)
                    q_full[d].extend(q)
                    qd_full[d].extend(qd)
                    qdd_full[d].extend(qdd)
            else:
                for d in range(m):
                    q, qd, qdd = self.eval_quintic(coeffs[d][i], t_seg)
                    q_full[d].extend(q)
                    qd_full[d].extend(qd)
                    qdd_full[d].extend(qdd)
        
        return np.array(t_full), np.array(q_full), np.array(qd_full), np.array(qdd_full)

    def compute_velocities_ts(self, waypoints, start_vel=None, end_vel=None, pause_time=0.001):
        """
        Compute waypoint velocities and time points with handling for identical (stationary) waypoints.
        - Automatically inserts a small pause_time between identical waypoints.
        - Ensures stable finite-difference velocity estimation.
        """
        pos = waypoints[:3].T
        n = pos.shape[0]

        dist = np.linalg.norm(np.diff(pos, axis=0), axis=1)

        for i in range(len(dist)):
            if dist[i] < 1e-6:
                dist[i] = 0.0

        total_dist = np.sum(dist)
        dist_ratio = np.insert(np.cumsum(dist), 0, 0)
        if total_dist > 0:
            dist_ratio /= dist_ratio[-1]
        else:
            dist_ratio = np.linspace(0, 1, n)

        time_points = dist_ratio * self.t_traj

        for i in range(1, len(dist) + 1):
            if np.linalg.norm(pos[i] - pos[i - 1]) < 1e-6:
                time_points[i:] += pause_time

        velocities = np.zeros_like(waypoints[:3])
        for i in range(1, waypoints[:3].shape[1] - 1):
            dt = time_points[i + 1] - time_points[i - 1]
            if dt < 1e-8:
                velocities[:, i] = 0
            else:
                dq = waypoints[:3][:, i + 1] - waypoints[:3][:, i - 1]
                velocities[:, i] = dq / dt
                velocities[:, 0] = -velocities[:, i-1]
                velocities[:, -1] = -velocities[:, i-1]

        # Start and End Velocities
        if start_vel is not None:
            velocities[:, 0] = start_vel
        if end_vel is not None:
            velocities[:, -1] = end_vel
    
        scale = self.scale_waypoint_vel
        velocities *= scale

        return time_points, velocities
    
    def compute_velocities_js(self, waypoints, start_vel=None, end_vel=None, pause_time=0.5):
        """
        Compute waypoint velocities and time points with handling for identical (stationary) waypoints.
        - Automatically inserts a small pause_time between identical waypoints.
        - Ensures stable finite-difference velocity estimation.
        """
        pos = waypoints.T
        n = pos.shape[0]

        dist = np.linalg.norm(np.diff(pos, axis=0), axis=1)

        for i in range(len(dist)):
            if dist[i] < 1e-6:
                dist[i] = 0.0

        total_dist = np.sum(dist)
        dist_ratio = np.insert(np.cumsum(dist), 0, 0)
        if total_dist > 0:
            dist_ratio /= dist_ratio[-1]
        else:
            dist_ratio = np.linspace(0, 1, n)

        time_points = dist_ratio * self.t_traj

        for i in range(1, len(dist) + 1):
            if np.linalg.norm(pos[i] - pos[i - 1]) < 1e-6:
                time_points[i:] += pause_time

        velocities = np.zeros_like(waypoints)
        for i in range(1, waypoints.shape[1] - 1):
            dt = time_points[i + 1] - time_points[i - 1]
            if dt < 1e-8:
                velocities[:, i] = 0
            else:
                dq = waypoints[:, i + 1] - waypoints[:, i - 1]
                velocities[:, i] = dq / dt
                velocities[:, 0] = -velocities[:, i-1]
                velocities[:, -1] = -velocities[:, i-1]

        # Start and End Velocities
        if start_vel is not None:
            velocities[:, 0] = start_vel
        if end_vel is not None:
            velocities[:, -1] = end_vel
    
        scale = self.scale_waypoint_vel
        velocities *= scale

        return time_points, velocities
    
    def q_spline(self, waypoints, time_step=100):
        """
        Generate smooth quintic spline trajectory from waypoints.
        Args:
            waypoints: list of [x, y, z, ...]
            time_step: number of interpolation points
            plot_time: total time duration for trajectory (seconds)
        """
        plot_time = self.t_traj
        
        waypoints_sepr = []
        for x in range(len(waypoints)):
            waypoints_sepr.append([waypoints[x][i:i + 3] for i in range(0, 4, 3)])

        points = [waypoints_sepr[p][0] for p in range(len(waypoints_sepr))]
        xyz_popoints = np.transpose(np.array(points))

        # time points (normalized to plot_time)
        t = np.linspace(0, plot_time, len(waypoints))

        x, y, z = xyz_popoints
        
        bc_type = 'periodic'
        spline_x = make_interp_spline(t, x, k=5, bc_type=bc_type)
        spline_y = make_interp_spline(t, y, k=5, bc_type=bc_type)
        spline_z = make_interp_spline(t, z, k=5, bc_type=bc_type)
        
        t_fine = np.linspace(0, plot_time, time_step)

        # spline position
        q_x = spline_x(t_fine)
        q_y = spline_y(t_fine)
        q_z = spline_z(t_fine)

        q = np.column_stack((q_x, q_y, q_z))

        self.t_fine = t_fine
        self.pos_x, self.pos_y, self.pos_z = q_x, q_y, q_z

        # Derivatives 
        self.vel_x = spline_x.derivative(1)(t_fine)
        self.vel_y = spline_y.derivative(1)(t_fine)
        self.vel_z = spline_z.derivative(1)(t_fine)

        self.acc_x = spline_x.derivative(2)(t_fine)
        self.acc_y = spline_y.derivative(2)(t_fine)
        self.acc_z = spline_z.derivative(2)(t_fine)

        self.jerk_x = spline_x.derivative(3)(t_fine)
        self.jerk_y = spline_y.derivative(3)(t_fine)
        self.jerk_z = spline_z.derivative(3)(t_fine)

        return q
    
    def q_spline_js(self, joint_angles, time_step=100):
        """
        Generate smooth quintic spline trajectory from waypoints.
        Args:
            waypoints: list of [x, y, z, ...]
            time_step: number of interpolation points
            plot_time: total time duration for trajectory (seconds)
        """
        plot_time = self.t_traj

        joint_qt = np.transpose(np.array(joint_angles))

        t = np.linspace(0, plot_time, len(joint_angles))

        bc_type = 'periodic'
        trajectories = [make_interp_spline(t, joint_qt[j], k=5, bc_type=bc_type) for j in range(len(joint_qt))]
        
        t_fine = np.linspace(0, plot_time, time_step)

        jq = np.array([traj(t_fine) for traj in trajectories])
        jq_vel = np.array([traj(t_fine, 1) for traj in trajectories])
        jq_acc = np.array([traj(t_fine, 2) for traj in trajectories])
        jq_jerk = np.array([traj(t_fine, 3) for traj in trajectories])

        self.t_fine = t_fine

        self.jq = jq
        self.jq_vel = jq_vel
        self.jq_acc = jq_acc
        self.jq_jerk = jq_jerk

        positions = np.array(np.transpose(jq))

        return positions
    
    def create_traj_jointspace(self, waypoints, xyz_mask=None, n_samples=100):
            waypoints_split = []
            for x in range(len(waypoints)):
                waypoints_split.append([waypoints[x][i:i + 3] for i in range(0, 4, 3)])

            xyz = []

            for i in range(len(waypoints_split)):
                xyz.append(waypoints_split[i][0])

            euler_angles = []

            eular_deg = self.model.euler_in_deg

            for e in range(len(waypoints_split)):
                if eular_deg: 
                    euler_angles.append(np.array([(r / 180) * np.pi for r in waypoints_split[e][1]]))
                else:
                    euler_angles.append(waypoints_split[e][1])
           
            targets = []
            for pos, rot in zip(xyz, euler_angles):
                T = np.concatenate([pos, rot])
                targets.append(T)

            self.fk.compute(np.zeros(len(self.model.args)))
            joint_angles = [self.ik.solve(targets[i], mask=xyz_mask) for i in range(len(targets))]

            jnt_conf = np.array(joint_angles).T
            time_points, velocities, = self.compute_velocities_js(waypoints=jnt_conf)
            self.velocities = velocities

            tr_type = getattr(self, 'tr_type', 'qu')  # default to quintic

            if tr_type == 'cu':
                coeffs = self.cubic_trajectory_nd(
                    waypoints=jnt_conf,
                    time_points=time_points,
                    velocities=velocities
                )
                eval_func = self.eval_cubic 
                t, jq, jqd, jqdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
                
                self.t_fine = t
                self.jq = jq
                self.jq_vel = jqd
                self.jq_acc = jqdd
                self.jq_jerk = None

                joint_trajectory = np.transpose(jq)
                return [joint_trajectory]
            elif tr_type == 'spl':
                joint_trajectory = self.q_spline_js(joint_angles=joint_angles, time_step=n_samples, xyz_mask=xyz_mask)
                return [joint_trajectory]
            else:
                coeffs = self.quintic_trajectory_nd(
                    waypoints=jnt_conf,
                    time_points=time_points,
                    velocities=velocities
                )  
                eval_func = self.eval_quintic
                t, jq, jqd, jqdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
                
                self.t_fine = t
                self.jq = jq
                self.jq_vel = jqd
                self.jq_acc = jqdd
                self.jq_jerk = None
                
                joint_trajectory = np.transpose(jq)
                return [joint_trajectory]
            
    def create_traj_taskspace(self, way_points, xyz_mask=None, n_samples=100):
        waypoints = np.array(way_points).T
        time_points, velocities, = self.compute_velocities_ts(waypoints=waypoints)

        self.velocities = velocities

        waypoints_split = []
        wayp =  waypoints.T # Transposed to default format

        for x in range(len(wayp)):
            waypoints_split.append([wayp[x][i:i + 3] for i in range(0, 4, 3)])

        xyz = []
        for i in range(len(waypoints_split)):
            xyz.append(waypoints_split[i][0])

        euler_angles = []
        eular_deg = self.model.euler_in_deg

        for e in range(len(waypoints_split)):
            if eular_deg: 
                euler_angles.append(np.array([(r / 180) * np.pi for r in waypoints_split[e][1]]))
            else:
                euler_angles.append(waypoints_split[e][1])

        # polynomial method
        tr_type = getattr(self, 'tr_type', 'qu')  # default to quintic

        if tr_type == 'cu':
            coeffs = self.cubic_trajectory_nd(
                waypoints=waypoints[:3],
                time_points=time_points,
                velocities=velocities
            )
            eval_func = self.eval_cubic 
            t, q, qd, qdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
            
            positions = np.transpose(q)
            self.t_fine = t

            self.pos_x = q[0]
            self.pos_y = q[1]
            self.pos_z = q[2]

            self.vel_x = qd[0]
            self.vel_y = qd[1]
            self.vel_z = qd[2]

            self.acc_x = qdd[0]
            self.acc_y = qdd[1]
            self.acc_z = qdd[2]

        elif tr_type == 'spl':
            q = self.q_spline(way_points, time_step=n_samples)
            positions = np.array(q)
        else:
            coeffs = self.quintic_trajectory_nd(
                waypoints=waypoints[:3],
                time_points=time_points,
                velocities=velocities
            )  
            eval_func = self.eval_quintic
            t, q, qd, qdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
           
            positions = np.transpose(q)
            self.t_fine = t

            self.pos_x = q[0]
            self.pos_y = q[1]
            self.pos_z = q[2]

            self.vel_x = qd[0]
            self.vel_y = qd[1]
            self.vel_z = qd[2]

            self.acc_x = qdd[0]
            self.acc_y = qdd[1]
            self.acc_z = qdd[2]
        
        # Convert Euler angles to quaternions
        quats = R.from_euler('xyz', euler_angles).as_quat(canonical=False)
        rotations = R.from_quat(quats)

        t_points = [t for t in range(0, len(wayp))]

        # SLERP setup
        key_times = t_points

        slerp = Slerp(key_times, rotations)

        # Interpolation times
        times = np.linspace(0, len(wayp)-1, len(positions))

        interp_rots = slerp(times).as_quat(canonical=False)
        eular = R.from_quat(interp_rots).as_euler('xyz', degrees=False)

        roll = [eular[i][2] for i in range(len(eular))]
        pitch = [eular[i][1] for i in range(len(eular))]
        yaw = [eular[i][0] for i in range(len(eular))]

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        poses = []
        for pos, rot in zip(positions, eular):
            T = np.concatenate([pos, rot])
            poses.append(T)
        
        joint_angles = self.wayp_to_joint_angle(poses, xyz_mask=xyz_mask)
        self.trajectory = joint_angles
        return joint_angles
    
    def create_trajectory(self, waypoints, traj_method='ts', xyz_mask=None, n_samples=100):
        if len(waypoints) < 2:
            raise ValueError(f"Too few target poses for robot :{self.model.robot_name}: expected a sequence start and end goal poses")
        self.traj_method = traj_method
        if traj_method == 'js':
            joint_traj = self.create_traj_jointspace(waypoints=waypoints, xyz_mask=xyz_mask, n_samples=n_samples)
            self.trajectory = joint_traj
            self.model.jnt_configs = joint_traj
            if self.model.joint_lim_enable:
                self.model.check_limits(joint_traj)
            return joint_traj
        elif traj_method == 'ts':
            ts_traj = self.create_traj_taskspace(way_points=waypoints, xyz_mask=xyz_mask, n_samples=n_samples)
            self.trajectory = ts_traj
            self.model.jnt_configs = ts_traj
            if self.model.joint_lim_enable:
                self.model.check_limits(ts_traj)
            return ts_traj
        else:
            raise ValueError(f"Unsupported trajectory method: {traj_method}")
        
    def joint_control(self, joint_poses, n_samples=100, rads=False):
        rads_to_deg = []

        self.traj_method = "js"

        j_type = self.model.get_joint_type()
        joint_p = []

        if rads:
            for i in range(len(joint_poses)):
                for j in range(len(joint_poses[i])):
                    if j_type[j] == "r":
                        rads_to_deg.append((joint_poses[i][j] / np.pi)*180)
                    elif j_type[j] == "p":
                        rads_to_deg.append(joint_poses[i][j])
            rads_to_deg_split = np.split(np.array(rads_to_deg), len(joint_poses))
            _joint_poses = rads_to_deg_split
           
        else:
            _joint_poses = joint_poses
            
        for a in range(len(_joint_poses)):
            for b in range(len(_joint_poses[a])):
                if j_type[b] == "r":
                    joint_p.append((_joint_poses[a][b] / 180) * np.pi)
                elif j_type[b] == "p":
                    joint_p.append(_joint_poses[a][b])

        joint_p_split = np.split(np.array(joint_p), len(_joint_poses))

        jnt_conf = np.array(joint_p_split).T
        time_points, velocities, = self.compute_velocities_js(waypoints=jnt_conf)
        self.velocities = velocities

        # polynomial method
        tr_type = getattr(self, 'tr_type', 'qu')  # default to quintic

        if tr_type == 'cu':
            coeffs = self.cubic_trajectory_nd(
                waypoints=jnt_conf,
                time_points=time_points,
                velocities=velocities
            )
            eval_func = self.eval_cubic 
            t, jq, jqd, jqdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
            
            self.t_fine = t
            self.jq = jq
            self.jq_vel = jqd
            self.jq_acc = jqdd
            self.jq_jerk = None
            joint_trajectory = np.transpose(jq)

        elif tr_type == 'spl':
            joint_trajectory = self.q_spline_js(joint_angles=joint_p_split, time_step=n_samples)
        else:
            coeffs = self.quintic_trajectory_nd(
                waypoints=jnt_conf,
                time_points=time_points,
                velocities=velocities
            )  
            eval_func = self.eval_quintic
            t, jq, jqd, jqdd = self.evaluate_full_trajectory(coeffs, time_points=time_points, n_samples=n_samples)
            
            self.t_fine = t
            self.jq = jq
            self.jq_vel = jqd
            self.jq_acc = jqdd
            self.jq_jerk = None
            joint_trajectory = np.transpose(jq)

        self.model.jnt_configs = [joint_trajectory]
        self.trajectory = [joint_trajectory]
        if self.model.joint_lim_enable:
            self.model.check_limits([joint_trajectory])

        return [joint_trajectory]
    
    @staticmethod
    def create_circle_traj(radius, cent, num_p=50):
        if type(radius) not in [int, float]:
            raise TypeError("radius must be of type integer or float")
        if type(num_p) not in [int]:
            raise TypeError("num_p must be of type integer or float")
        if type(cent) is not list:
            raise TypeError("center must be of type list")
        for val in cent:
            if type(val) not in [int, float]:
                raise TypeError("values in list must be of type int or float")
    
        center = np.array(cent)
        radius = radius
        num_points = num_p 
        time = np.linspace(0, 2 * np.pi, num_points)

        # Generate circular trajectory
        x = center[0] + radius * np.cos(time)
        y = center[1] + radius * np.sin(time)
        z = np.full_like(time, center[2])

        roll = center[5] + radius * time*0
        pitch = center[4] + radius * time*0
        yaw = center[3] + radius * time*0

        trajectory = np.vstack((x, y, z, yaw, pitch, roll)).T 
        return trajectory


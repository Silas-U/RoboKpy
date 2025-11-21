# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import numpy as np
from scipy.spatial.transform import Rotation as R

class InverseKinematics:
    def __init__(self, model, fk, jacobian, damp=1e-2):
        self.model = model
        self.fk = fk
        self.jacobian = jacobian
        self.damp = damp
        self.success = False
        self.initial_guess_val = None
        self.initial_guess_rads = False
        
    def initial_guess(self, val, rads=False):
        self.initial_guess_rads = rads
        self.initial_guess_val = val

    def init_guess(self):
        if self.initial_guess_val is None:
            num_of_joints = self.model.get_num_of_joints()
            self.initial_guess_val = np.zeros(num_of_joints)
        self.fk.compute(self.initial_guess_val, rads=self.initial_guess_rads)

    # -------------------------
    # Quaternion utilities
    # -------------------------

    @staticmethod
    def normalize_quat(q):
        q = np.array(q, dtype=float)
        n = np.linalg.norm(q)
        return q if n < 1e-12 else q / n

    @staticmethod
    def quat_conjugate(q):
        # q = [x,y,z,w]
        return np.array([-q[0], -q[1], -q[2], q[3]])

    @staticmethod
    def quat_mul(q1, q2):
        # Hamilton product for quaternions [x,y,z,w]
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return np.array([x, y, z, w])

    @staticmethod
    def quat_to_rotvec(q):
        # robust — use SciPy
        return R.from_quat(q).as_rotvec()

    # -------------------------
    # Main IK solver
    # -------------------------
    def solve(self, target_position, mask=None, tol=1e-3, max_iter=500,
              rpy_deg=False, output_deg=False):
        
        if type(target_position) not in [np.ndarray, list]:
            raise TypeError(f"Expected a list of cartesian waypoints but got {type(target_position)}")
        if type(tol) not in [int, float]:
            raise TypeError("tolerance must be of type integer or float")
        if type(max_iter) not in [int, float]:
            raise TypeError("max_iter must be of type integer or float")
        
        print("\nIK:searching...")
        print("────────────────────────────────────────────────────────")
        print(f"{'Iter':>4} | {'Error':>12} | {'Δθ':>12}")
        print("────────────────────────────────────────────────────────")

        self.init_guess()

        if mask is None:
            mask = [1, 1, 1, 1, 1, 1]
        elif type(mask) not in [np.ndarray, list]:
            raise TypeError(f"mask must be of type list or ndarray. e.g: {[1, 1, 1, 1, 1, 1]}")

        mask_p = np.array(mask[:3], dtype=float)
        mask_r = np.array(mask[3:], dtype=float)

        # --------------------------------
        # Process target input
        # --------------------------------
        target = np.array(target_position, dtype=float)

        if target.size == 7:
            # [px,py,pz, qx,qy,qz,qw]
            p_desired = target[:3]
            q_desired = self.normalize_quat(target[3:])
        elif target.size == 6:
            # [px,py,pz, r,p,y]
            p_desired = target[:3]
            r_desired = np.deg2rad(target[3:]) if rpy_deg else target[3:]
            q_desired = R.from_euler("xyz", r_desired).as_quat()
        else:
            raise ValueError("Target must be length 6 (RPY) or 7 (quaternion)")

        q_desired = self.normalize_quat(q_desired)

        # --------------------------------
        # IK iteration loop
        # --------------------------------
        i = 0
        th = np.zeros(self.model.num_of_joints)
        final_conv_error = 0
        damp = self.damp

        while True:
            # FK current pose
            p_current = np.array(self.fk.get_target_xyz(), dtype=float)
            q_current = self.normalize_quat(
                np.array(self.fk.get_target_quart(), dtype=float)
            )

            # Position error
            e_position = (p_desired - p_current) * mask_p

            # Quaternion orientation error
            q_err = self.quat_mul(q_desired, self.quat_conjugate(q_current))
            e_orientation = self.quat_to_rotvec(q_err)
            e_orientation = e_orientation * mask_r

            # Final error vector
            error = np.concatenate((e_position, e_orientation))

            # Check convergence
            if np.linalg.norm(error) < tol:
                self.success = True
                break
            if i >= max_iter:
                self.success = False
                break

            # Jacobian and damped least squares
            try:
                J = np.asarray(self.jacobian.compute())  # 6×n

                # Weighted rows (masking)
                W = np.diag(np.hstack((mask_p, mask_r)))
                Jw = W @ J
                ew = W @ error

                JJt = Jw @ Jw.T
                y = np.linalg.solve(JJt + damp * np.eye(6), ew)
                d_theta = Jw.T @ y

            except Exception:
                d_theta = np.zeros(self.model.num_of_joints)

            # Apply update
            th += d_theta
            self.fk.compute(th, rads=True)

            # Logging
            if i % 10 == 0:
                print(f"{i:4d} | {np.linalg.norm(error):12.6e} | {np.linalg.norm(d_theta):12.6e}")

            final_conv_error = f"{np.linalg.norm(error):.6f}"
            i += 1

        # --------------------------------
        # Output
        # --------------------------------
        if self.success:
            print("────────────────────────────────────────────────────────")
            print(f"Converged in {i} iterations")
            print(f"Final error norm: {final_conv_error}")

            self.model.joint_states_deg = output_deg
            result = self.fk.get_joint_states(in_degrees=output_deg)

            # Reset FK to zero pose
            self.fk.compute(np.zeros(self.model.num_of_joints), rads=True)
            return result

        else:
            self.fk.compute(np.zeros(self.model.num_of_joints), rads=True)
            print("\nWarning!: solver failed to converge within maximum iterations.")
            print(f"Final error norm: {final_conv_error}")
            print("────────────────────────────────────────────────────────")
            return np.zeros(self.model.num_of_joints)


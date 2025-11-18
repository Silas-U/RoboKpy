"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
"""

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

    def solve(self, target_position, mask=None, tol=1e-3, max_iter=500, rpy_deg=False, output_deg=False):
        
        print("\nIK:searching...")
        print("────────────────────────────────────────────────────────")
        print(f"{'Iter':>4} | {'Error':>12} | {'Δθ':>12}")
        print("────────────────────────────────────────────────────────")

        self.init_guess()
        if mask is None:
            mask = [1, 1, 1, 1, 1, 1]
       
        TOL = tol
        IT_MAX = max_iter
        damp = self.damp
        
        th = np.zeros(self.model.num_of_joints)
        final_conv_error = 0

        mask_p = np.array(mask[:3])     
        mask_r = np.array(mask[-3:])

        p_desired = np.array(target_position[:3])
        
        if rpy_deg:
            r_desired = np.array((target_position[-3:] / 180) * np.pi)
        else:
            r_desired = np.array(target_position[-3:])
        
        i = 0

        while True:
            current_position = self.fk.get_target_xyz()
            current_orientation = self.fk.get_target_rpy()

            p_current = np.array(current_position)
            r_current = np.array(current_orientation)
            e_position = (p_desired - p_current)*mask_p

            q_desired = R.from_euler('xyz', r_desired*mask_r, degrees=False).as_quat(canonical=False)  # desired quaternion
            q_current = R.from_euler('xyz', r_current*mask_r, degrees=False).as_quat(canonical=False)  # current quaternion
            
            R_error = R.from_quat(q_desired) * R.from_quat(q_current).inv()
            e_orientation = R_error.as_rotvec()
            error = (np.concatenate((e_position, e_orientation)))

            if np.linalg.norm(error) < TOL:
                self.success = True
                break
            if i >= IT_MAX:
                self.success = False
                break

            try:
                jc = self.jacobian.compute()
                J = np.asarray(jc) # 6 x n
                JJt = J.dot(J.T)   # 6 x 6
                damped = JJt + (damp * np.eye(JJt.shape[0]))
                y = np.linalg.solve(damped, error)
                d_theta = J.T.dot(y)   
            except Exception:
                d_theta = np.zeros(self.model.num_of_joints)

            th += d_theta
            self.fk.compute(th, rads=True)

            step_norm = np.linalg.norm(d_theta)

            if i % 10 == 0:
                print(f"{i:4d} | {np.linalg.norm(error):12.6e} | {step_norm:12.6e}")

            final_conv_error = f"{np.linalg.norm(error):.6f}"

            i += 1

        if self.success:
            print("────────────────────────────────────────────────────────")
            print(f"Converged in {i} iterations")
            print(f"Final error norm: {final_conv_error}")
            if i == 0:
                self.model.joint_states_deg = output_deg
                j_states = self.fk.get_joint_states(in_degrees=output_deg)
            else:
                self.model.joint_states_deg = output_deg
                j_states = self.fk.get_joint_states(in_degrees=output_deg)
            self.fk.compute(np.zeros(self.model.num_of_joints), rads=True) 
            return j_states
        else:
            self.fk.compute(np.zeros(self.model.num_of_joints), rads=True)
            print("\nWarning!: solver failed to converge within maximum iterations.")
            print(f"Final error norm: {final_conv_error}")
            print("────────────────────────────────────────────────────────")
            # print("\nWarning!: the iterative algorithm has not reached convergence to the desired precision")
            return np.zeros(self.model.num_of_joints)
        
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

class Jacobian:
    def __init__(self, model, fk):
        self.model = model
        self.fk = fk
        self.jac = None
        self.singular_confs = []

    def compute(self):
        n = self.model.num_of_joints
        o_n = np.array(self.fk.get_j_origin(self.fk.transform_length()))
        Js_v = []
        Js_w = []
        for i in range(1, 2*n, 2):  # HTM pair indexing
            idx = (i-1)//2   # convert to joint index
            o_i = np.array(self.fk.get_j_origin(i))
            R_i = np.array(self.fk.get_r_matrix(i))
            z_i = R_i.dot(np.array([0.0,0.0,1.0]))
            if self.model.joint_type_info[idx] == 'r':
                jv = np.cross(z_i, (o_n - o_i))
                jw = z_i
            else:
                jv = z_i
                jw = np.zeros(3)
            Js_v.append(jv)
            Js_w.append(jw)
        J = np.vstack((np.array(Js_v).T, np.array(Js_w).T))
        self.jac = J
        return J

    def singular_conf_check(self, threshold=1e-5):
        J = self.compute()
        rank = np.linalg.matrix_rank(J, tol=threshold)
        jnt_config = self.fk.get_joint_states(in_degrees=True)

        if rank < min(J.shape):
            self.singular_confs.append(jnt_config)
            print(f"Singularity detected at config {jnt_config}: jac rank = {rank} < {min(J.shape)}\n")

        if self.singular_confs  is None:
            print("No singularities detected.")
        else:
            print(f"Total Singular Configurations Found: {len(self.singular_confs)}")

    def rank(self):
        if self.jac is None:
            raise ValueError('Jacobian not computed yet')
        rank = np.linalg.matrix_rank(self.jac)
        return rank

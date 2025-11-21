# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import math as m
import numpy as np
from functools import reduce
from scipy.spatial.transform import Rotation as R
from .utils import clamp, validate_keys

class ForwardKinematics:
    def __init__(self, model):
        self.model = model

    def _group_dh(self, joint_vars, rads=False):
        args = self.model.args
        if len(joint_vars) != len(args):
            raise IndexError(f"Expected {len(args)} joint values but got {len(joint_vars)}")
        
        dh_params = []
        for i in range(len(args)):
            data = args[i].copy()
            jtype = data['joint_type']
            if jtype == 'r':
                val = float(joint_vars[i])
                if not rads:
                    val = np.deg2rad(val)
                data['theta'] = val
            else: # prismatic
                data['joint_offset'] = float(joint_vars[i])
            dh_params.append([data['joint_name'], data['joint_type'], data['link_length'],
                              data['twist'], data['joint_offset'], data['theta'], data.get('offset',0.0)])
        self.model.dh_param_grouped_list = dh_params
        self.model.num_of_joints = len(dh_params)
        self.model.joint_type_info = [d[1] for d in dh_params]
        return dh_params
    

    def compute(self, joint_vars, rads=False):
        if type(joint_vars) not in [np.ndarray, list]:
            raise TypeError(f"Expected a list of joint_vars but got {type(joint_vars)}")
        dh_params = self._group_dh(joint_vars, rads)
        t_matrices = []
        for item in dh_params:
            link_length = float(item[2])
            joint_offset = float(item[4])
            theta = float(item[5])
            offset = float(item[6])

            if self.model.link_twist_in_rads:
                twist = float(item[3])
            else:
                twist = (float(item[3]) / 180) * m.pi

            htm_joint_offset = [[1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, joint_offset + offset],
                                [0.0, 0.0, 0.0, 1.0]]

            htm_link_length = [[np.cos(theta), -np.sin(theta) * np.cos(twist),
                                np.sin(theta) * np.sin(twist), link_length * np.cos(theta)],
                               [np.sin(theta), np.cos(theta) * np.cos(twist),
                                -np.cos(theta) * np.sin(twist), link_length * np.sin(theta)],
                               [0.0, np.sin(twist), np.cos(twist), 0],
                               [0.0, 0.0, 0.0, 1.0]]

            t_matrices.append(np.array(htm_joint_offset))
            t_matrices.append(np.array(htm_link_length))

        self.model.homogeneous_t_matrices = t_matrices
        return t_matrices

    @staticmethod
    def _pair_multiply(htmxes):
        if len(htmxes) % 2 != 0:
            raise ValueError("HT matrices count must be even")
        products = []
        for i in range(0, len(htmxes), 2):
            products.append(np.dot(htmxes[i], htmxes[i+1]))
        return products

    def get_transforms(self, stop_index=None, real=False):
        htm = self.model.homogeneous_t_matrices
        if stop_index is None:
            stop_index = len(self._pair_multiply(htm))
        if stop_index < 1 or stop_index > len(htm):
            raise IndexError("stop_index out of range")
        new = [htm[i] for i in range(stop_index)]
        result = reduce(np.dot, np.array(new))
        np.set_printoptions(suppress=True) ###
        return result

    def get_htm(self):
        T = self.get_transforms(stop_index=self.transform_length())
        return T
    
    def get_tcp(self):
        T = self.get_transforms(stop_index=self.transform_length())
        return [T[i][3] for i in range(3)]

    def get_j_origin(self, index):
        T = self.get_transforms(stop_index=index)
        return [T[i][3] for i in range(3)]

    def get_r_matrix(self, index):
        T = self.get_transforms(stop_index=index)
        Rm = [[T[i][j] for j in range(3)] for i in range(3)]
        return Rm
    
    def transform_length(self):
        return len(self.model.homogeneous_t_matrices)
        
    def SE3(self, T, deg=False, merge_res=False):
        try:
            if type(T) is not np.ndarray:
                for item in T:
                    if type(item) not in [int, float]:
                        raise TypeError("input must be of type integer, float or "
                                        "numpy.ndarray with shape (3, 3) or (N, 3, 3)")
            position = T[:3, 3]
            rotation_matrix = T[:3, :3]
            r = R.from_matrix(rotation_matrix)
            quats = r.as_quat(canonical=False)
            
            self.quartenion = quats
            self.position = position

            return np.concatenate([position, quats])
        except ValueError as e:
            print(f"Error: {e}")

    def get_target(self):
        tar = self.SE3(self.get_transforms(self.transform_length()), merge_res=True)
        return tar

    def get_target_xyz(self):
        self.SE3(self.get_transforms(self.transform_length()))
        pos_xyz = self.position
        return pos_xyz
    
    def get_target_quart(self):
        self.SE3(self.get_transforms(self.transform_length()))
        rpy = self.quartenion
        return rpy
    
    def get_joint_states(self, in_degrees=False):
        joint_state = []
        for i in range(len(self.model.get_dh_params())):
            if self.model.get_dh_params()[i][1] == "r":
                val = float(self.model.get_dh_params()[i][5])
                joint_state.append(np.rad2deg(val) if in_degrees else val)
            else:  # prismatic
                joint_state.append(float(self.model.get_dh_params()[i][4]))
        return joint_state
    
    @staticmethod
    def __mul_htm_jo_htm_ll(htmxes):
        res_lst = []
        if len(htmxes) % 2 != 0:
            raise ValueError("The number of ht matrices must be even to form pairs")
        for i in range(0, len(htmxes), 2):
            product = np.dot(htmxes[i], htmxes[i+1])
            res_lst.append(product)
        return res_lst
    
    def get_jtj_transform(self, index=None, real=False):
        if index is None:
            index = 1
        htmxes = self.model.homogeneous_t_matrices
        JTJ_transformations = self.__mul_htm_jo_htm_ll(htmxes)
        new = [JTJ_transformations[i] for i in range(index)]
        if real:
            np.set_printoptions(suppress=True)
        return np.array(new)
    
    def __get_transform(self, htmxes, stop_index=0, real=False):
        try:
            if len(htmxes) == 0:
                raise IndexError(f"error: no initial values for fk analysis"
                                 f"for {self.model.robot_name}")

            new = [htmxes[i] for i in range(stop_index)]
            result = reduce(np.dot, np.array(new))
            if real:
                np.set_printoptions(suppress=True)
            return result
        except ValueError as e:
            print(f"Error: {e}")

    def get_transform(self, stop_index=1, real=False):
        try:
            htmxes = self.model.homogeneous_t_matrices
            res = self.__mul_htm_jo_htm_ll(htmxes)

            if stop_index <= 0:
                raise IndexError(f"error: transformation_index range from 1 - {len(res)} for {self.model.robot_name}")
            elif stop_index > len(res):
                raise IndexError(f"expected values from 1 - {len(res)}, but {stop_index} was provided")

            if len(res) == 0:
                raise IndexError(f"error: no initial values for fk analysis"
                                 f"for {self.model.robot_name}")

            new = [res[i] for i in range(stop_index)]
            result = reduce(np.dot, np.array(new))
            if real:
                np.set_printoptions(suppress=True)
            return result
        except ValueError as e:
            print(f"Error: {e}")

    def get_tranformations(self):
        htmxes = self.model.homogeneous_t_matrices
        JTJ_transformations = self.__mul_htm_jo_htm_ll(htmxes)

        BTJ_transformations = []
        for t in range(len(JTJ_transformations)):
             BTJ_transformations.append(self.__get_transform(JTJ_transformations, t+1, real=True))
        return np.array(BTJ_transformations)

    def show_tranformations(self):
        htmxes = self.model.homogeneous_t_matrices
        JTJ_transformations = self.__mul_htm_jo_htm_ll(htmxes)

        BTJ_transformations = []
        for t in range(len(JTJ_transformations)):
             BTJ_transformations.append(self.__get_transform(JTJ_transformations, t+1, real=True))

        print("====== Joint to Joint Transformations ====== \n")
        for index, res in enumerate(JTJ_transformations):
            print(f"Transformations {index}_{index+1}:\n{res}", "\n")
        
        print("====== Base to Joint Transformations ====== \n")
        for index, trns in enumerate(BTJ_transformations):
            print(f"Transformations {0}_{index+1}: \n{trns}", "\n")

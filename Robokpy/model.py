# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import numpy as np
from .utils import validate_keys

class RobotModel:
    """Holds robot DH params and state used by other modules."""
    def __init__(self, args, robot_name, link_twist_in_rads=False, use_jnt_lim=False):
        if not isinstance(robot_name, str):
            raise ValueError("robot_name must contain only letters")
        if len(args) == 0:
            raise ValueError("DH params list cannot be empty")
        for x in args:
            validate_keys(x)
        self.args = args
        self.robot_name = robot_name
        self.link_twist_in_rads = link_twist_in_rads
        self.joint_lim_enable = use_jnt_lim
        self.dh_param_grouped_list = []
        self.homogeneous_t_matrices = []
        self.joint_type_info = []
        self.num_of_joints = 0
        self.joint_limits = []
        self.quaternion = None
        self.jnt_configs = None
        self.invalid_jnt_configs = []
        self.joint_states_deg = False
        self.euler_in_deg = False

    def set_eular_in_deg(self, state):
        self.euler_in_deg = state

    def get_dh_table(self):
        dh_params_list = []
        for x in range(len(self.args)):
            for items in validate_keys(self.args[x]).items():
                dh_params_list.append(items)
        dh_table = np.array(np.split(np.array(dh_params_list), len(self.args)))
        return dh_table
    
    def get_dh_params(self):
        return self.dh_param_grouped_list

    def get_joint_type(self):
        jt = [validate_keys(self.args[i])['joint_type'] for i in range(len(self.args))]
        return jt

    def get_sum_link_lengths(self):
        link_l = [validate_keys(self.args[i]).get('link_length') for i in range(len(self.args))]
        return sum(link_l)

    def get_joint_names(self):
        return [validate_keys(self.args[i]).get('joint_name') for i in range(len(self.args))]

    def get_robot_name(self):
        return self.robot_name
    
    def get_num_of_joints(self):
        return len(self.args)
    
    def l_twist_in_rads(self):
        return self.link_twist_in_rads
    
    def structure(self):  # prints the joint type structure of the robot
        struc = self.get_joint_type()
        str_txt = ''.join(struc)
        print(str_txt.upper())

    def set_joint_limits(self, limits: dict):
        num_joints = len(self.args)
        if not all(k in limits for k in ("min", "max")):
            raise ValueError("Limits dictionary must have 'min' and 'max' keys")

        if len(limits["min"]) != num_joints:
            raise ValueError(f"Expected {num_joints} joint limits @min, got {len(limits['min'])}")
        
        if len(limits["max"]) != num_joints:
            raise ValueError(f"Expected {num_joints} joint limits @max, got {len(limits['max'])}")

        lower_limits = [limits["min"][f"j{i+1}"] for i in range(num_joints)]
        upper_limits = [limits["max"][f"j{i+1}"] for i in range(num_joints)]
        self.joint_limits = (np.array(lower_limits), np.array(upper_limits))

    def get_joint_limits(self):
        return self.joint_limits

    def check_limits(self, q, stop_on_first=True):
        if len(self.joint_limits) == 0:
            raise ValueError("Expected joint limits min,max")

        q_min = self.joint_limits[0]
        q_max = self.joint_limits[1]

        """Check if joint configuration q is within limits."""
        for x in range(len(q)):
            for y in range(len(q[x])):
                conf = q[x][y]
                exceeded = False

                for i, val in enumerate(conf):
                    if val < q_min[i]:
                        print(f"Warning! Joint {i+1} below lower limit: {val:.4f} < {q_min[i]:.4f}")
                        exceeded = True
                        if stop_on_first:   # exit
                            break
                    elif val > q_max[i]:
                        print(f"Warning! Joint {i+1} above upper limit: {val:.4f} > {q_max[i]:.4f}")
                        exceeded = True
                        if stop_on_first:   # exit
                            break

                if exceeded:
                    self.invalid_jnt_configs.append(conf)

    def get_jnt_configs(self):
        conf = []
        if self.jnt_configs is None:
            self.jnt_configs = [[np.zeros(self.num_of_joints)]]
        for x in range(len(self.jnt_configs)):
            for y in range(len(self.jnt_configs[x])):
                conf.append(self.jnt_configs[x][y])
        return conf
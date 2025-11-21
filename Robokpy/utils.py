# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

ALLOWED_KEYS = ['joint_name', 'joint_type', 'link_length', 'twist', 'joint_offset', 'theta', 'offset']

def validate_keys(data):
    if not all(key in ALLOWED_KEYS for key in data):
        raise KeyError(f"DH table contains invalid joint parameter names. Valid names: {ALLOWED_KEYS}")
    if len(data) < len(ALLOWED_KEYS):
        raise KeyError(f"Missing DH parameter in DH table. Valid names: {ALLOWED_KEYS}")
    return data

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def skew(vector):
    x, y, z = float(vector[0]), float(vector[1]), float(vector[2])
    return [[0.0, -z, y],
            [ z, 0.0, -x],
            [-y, x, 0.0]]

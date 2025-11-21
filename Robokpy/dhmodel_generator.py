# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import os
from pathlib import Path

MODEL_FILE_NAME = "Model.py"

TARGET_DIR = Path.cwd()
MODEL_FILE_PATH = TARGET_DIR / MODEL_FILE_NAME


TEMPLATE = r'''
"""
Auto-generated DH Model file.
Modify freely. Add new robot models by inheriting from BaseDHModel.

Created by RoboKpy
"""

import numpy as np
from Robokpy.dhmodel_base import BaseDHModel


class DHModel(BaseDHModel):

    # register your robot models here:
    MODELS = {    
        'Puma561': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0,   'twist': 90.0,   'joint_offset': 0.234,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.2318, 'twist': 0.0,  'joint_offset': 0.04909, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j3', 'joint_type': 'r', 'link_length': 0.0203, 'twist': 90.0, 'joint_offset': -0.05, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0.,  'twist': -90.0,  'joint_offset': 0.3318,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j5', 'joint_type': 'r', 'link_length': 0,   'twist': 90.0, 'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j6', 'joint_type': 'r', 'link_length': 0,   'twist': 0.0,  'joint_offset': 0.05625, 'theta': 0.0, 'offset': 0.0},
        ],

        'Puma560': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0,   'twist': 90.0,   'joint_offset': 0.0,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.4318, 'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j3', 'joint_type': 'r', 'link_length': 0.0203, 'twist': -90.0, 'joint_offset': 0.15, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0.,  'twist': 90.0,  'joint_offset': 0.4318,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j5', 'joint_type': 'r', 'link_length': 0,   'twist': -90.0, 'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j6', 'joint_type': 'r', 'link_length': 0,   'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
        ],

        'Cobra600': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0.325,  'twist': 0.0, 'joint_offset': 0.387,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.275, 'twist': np.pi,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j3', 'joint_type': 'p', 'link_length': 0, 'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0., 'twist': 0.0,  'joint_offset': 0.0,  'theta': 0.0, 'offset': 0.0},
        ],

        'Cylindrical': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0, 'twist': 0.0, 'joint_offset': 0.2, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'p', 'link_length': 0, 'twist': 90.0, 'joint_offset': 0.1, 'theta': 0.0, 'offset': 0.4},
            {'joint_name': 'j3', 'joint_type': 'p', 'link_length': 0, 'twist': 90.0, 'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j4', 'joint_type': 'p', 'link_length': 0, 'twist': 0.0, 'joint_offset': 0.1, 'theta': 0.0, 'offset': 0.0},
        ],

        'UR10': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0,  'twist': 90.0, 'joint_offset': 0.17,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.425, 'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.1038},
            {'joint_name': 'j3', 'joint_type': 'r', 'link_length': 0.39225, 'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': -0.1038},
            {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0, 'twist': 90.0,  'joint_offset': 0.0615,  'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j5', 'joint_type': 'r', 'link_length': 0,  'twist': -90.0, 'joint_offset': 0.0515, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j6', 'joint_type': 'r', 'link_length': 0,  'twist': 0.0,  'joint_offset': 0.0423, 'theta': 0.0, 'offset': 0.0},
        ],

        '6dof': [
            {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0.05, 'twist': np.pi/2,  'joint_offset': 0.105, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.14, 'twist': 0.0,      'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j3', 'joint_type': 'r', 'link_length': 0.17, 'twist': np.pi/2,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0.0,  'twist': -np.pi/2, 'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j5', 'joint_type': 'r', 'link_length': 0.0,  'twist': np.pi/2,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
            {'joint_name': 'j6', 'joint_type': 'r', 'link_length': 0.0,  'twist': 0.0,      'joint_offset': 0.02, 'theta': 0.0,  'offset': 0.0},
        ],

        '2dof':  [
            {"joint_name": "j1", "joint_type": "r", "link_length": 0.4, "twist": 0, "joint_offset": 0.2, "theta": 0, 'offset': 0.0},
            {"joint_name": "j2", "joint_type": "r", "link_length": 0.4, "twist": 0, "joint_offset": 0, "theta": 0, 'offset': 0.0}
        ],
    }

'''


def generate_model_file():
    """Creates external Model.py if missing."""
    TARGET_DIR.mkdir(parents=True, exist_ok=True)

    if not MODEL_FILE_PATH.exists():
        with open(MODEL_FILE_PATH, "w") as f:
            f.write(TEMPLATE)

        print(f"[RoboKpy] Created Model file at: {MODEL_FILE_PATH}")

    return MODEL_FILE_PATH

"""
Author: Silas Udofia
Date: 2024-08-02
GitHub: https://github.com/Silas-U/RoboKpy/tree/main

Robot DH Parameter Library

This module stores predefined Denavit-Hartenberg (DH) models for popular robots
and utilities to retrieve, list, and pretty-print them.

Licensed under the Apache License, Version 2.0
"""

import numpy as np
import pandas as pd


class ModelNotFoundError(Exception):
    """Raised when requested robot model is not found in the library."""


class DHModel:

    # register your robot models here:
    MODELS = {
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

    'Baxter': [
        {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0.069,   'twist': -90.0,   'joint_offset': 0.27,  'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0, 'twist': 90.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0},
        {'joint_name': 'j3', 'joint_type': 'r', 'link_length': 0.069, 'twist': -90.0, 'joint_offset': 0.364, 'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j4', 'joint_type': 'r', 'link_length': 0,  'twist': 90.0,  'joint_offset': 0.0,  'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j5', 'joint_type': 'r', 'link_length': 0.01,   'twist': -90.0, 'joint_offset': 0.374, 'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j6', 'joint_type': 'r', 'link_length': 0,   'twist': 90.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j7', 'joint_type': 'r', 'link_length': 0,   'twist': 0.0,  'joint_offset': 0.28, 'theta': 0.0, 'offset': 0.0},
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

    'scara':    [
        {'joint_name': 'j1', 'joint_type': 'r', 'link_length': 0.2, 'twist': 0.0, 'joint_offset': 0.2,  'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j2', 'joint_type': 'r', 'link_length': 0.2, 'twist': np.pi,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
        {'joint_name': 'j3', 'joint_type': 'p', 'link_length': 0, 'twist': 0.0,  'joint_offset': 0.0, 'theta': 0.0, 'offset': 0.0},
    ],
    }

    @classmethod
    def list_models(cls):
        return list(cls.MODELS.keys())

    @classmethod
    def get_model(cls, name: str):
        """
        Retrieve a DH model by name.
        Args:
            name (str): Model name (case-sensitive).
        Returns:
            list of dicts representing DH parameters.
        Raises:
            ModelNotFoundError: if the model is not available.
        """
        if name not in cls.MODELS:
            raise ModelNotFoundError(
                f"Model '{name}' not found. Available models: {', '.join(cls.list_models())}"
            )
        return cls.MODELS[name]

    @classmethod
    def search(cls, keyword: str):
        return [m for m in cls.MODELS if keyword.lower() in m.lower()]

    @classmethod
    def print_DH_table(cls, name: str):
        model = cls.get_model(name)
        df = pd.DataFrame(model)
        print(f"\nDH Parameters : {name}:\n")
        print(df.to_string(index=False))

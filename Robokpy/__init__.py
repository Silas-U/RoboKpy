# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

from .model import RobotModel
from .fk import ForwardKinematics
from .ik import InverseKinematics
from .jacobian import Jacobian
from .trajectory import TrajectoryPlanner
from .plotting import Plotter
from .mviz import VizModel
from .dhmodel_generator import generate_model_file
from .dhmodel_loader import load_user_models

# Auto-generate Model.py on package import
MODEL_FILE_PATH = generate_model_file()

# Load all user-defined models
USER_MODELS = load_user_models()

__all__ = ["USER_MODELS", "MODEL_FILE_PATH"]

class Init_Model:
    """High level API wrapping."""
    def __init__(self, dh_args, robot_name, twist_in_rads=False, use_jnt_lim=False, plt_model=False):
        self.model = RobotModel(dh_args, robot_name, twist_in_rads, use_jnt_lim)
        self.fk = ForwardKinematics(self.model)
        self.jac = Jacobian(self.model, self.fk)
        self.ik = InverseKinematics(self.model, self.fk,  self.jac)
        if plt_model:
            self.mviz = VizModel(self.model, self.fk)
        self.traj = TrajectoryPlanner(self.model, self.fk, self.ik, self.jac)
        self.plotter = Plotter(self.model, self.fk, self.traj)

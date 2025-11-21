# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
# """

import importlib.util
from .dhmodel_generator import MODEL_FILE_PATH


def load_user_models():
    """
    Loads user-defined models dynamically from Model.py
    Returns a dictionary of model_name: ModelInstance
    """
    spec = importlib.util.spec_from_file_location("user_models", MODEL_FILE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module

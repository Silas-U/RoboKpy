# """
# Author: Silas Udofia
# Date: 2024-08-02
# GitHub: https://github.com/Silas-U/RoboKpy/tree/main

# Robot DH Parameter Library

# This module stores predefined Denavit-Hartenberg (DH) models for popular robots
# and utilities to retrieve, list, and pretty-print them.

# Licensed under the Apache License, Version 2.0
# """

import pandas as pd


class ModelNotFoundError(Exception):
    """Raised when requested robot model is not found in the library."""

class BaseDHModel:
    def __init__(self):
        pass

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
    def print_dh_table(cls, name: str):
        model = cls.get_model(name)
        df = pd.DataFrame(model)
        print(f"\nDH Parameters : {name}:\n")
        print(df.to_string(index=False))

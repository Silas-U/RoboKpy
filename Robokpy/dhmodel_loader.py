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

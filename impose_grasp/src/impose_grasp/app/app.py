import os
import json
import numpy as np


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NumpyArrayEncoder, self).default(obj)

def write_settings(path_to_settings):
    with open(path_to_settings, 'w') as F:
        json.dump(App().settings, F, indent=2, cls=NumpyArrayEncoder)


def read_settings(path_to_settings):
    with open(path_to_settings, 'r') as F:
        data = json.load(F)
    App().settings = data


def register_settings(name, new_set):
    set = App().settings
    set[name] = new_set


class App:
    """ App Singleton """
    _instance = None
    window = None
    scene = None
    pov_scene = None
    settings = {}

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(App, cls).__new__(cls, *args, **kwargs)
            settings_path = os.path.join(os.getcwd(), 'src', 'impose_grasp', 'app', 'settings.json')
            read_settings(settings_path)
        return cls._instance


import os
import json
import numpy as np

from typing import List
from math import pi, cos, sin

from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

class Grasps:
    def __init__(self) -> None:
        self.rel_poses: List[np.ndarray] = []
        self.abs_poses: List[np.ndarray] = []
        self.widths: List [float] = []
        self.power_gr: List [bool] = []
        
    def load_from_file(self, obj_name):
        MODELS_PATH = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models")
        grasps_path = os.path.join(MODELS_PATH, obj_name, "gripping_poses.json")

        if os.path.isfile(grasps_path):
            with open(grasps_path) as F:
                json_load = json.load(F)
            for x in json_load:     
                self.rel_poses.append(eval('np.array(' + x["pose"] + ')'))
                self.widths.append(x["width"])
    
    def set_rel_poses(self, rel_poses: List[np.ndarray]):
        self.rel_poses = rel_poses

    def set_widths(self, widths: List[float]):
        self.widths = widths

    def set_power_gr(self, power_gr: List [bool]):
        self.power_gr = power_gr

    def set_abs_poses(self, obj_pose:np.ndarray):
        self.abs_poses = [obj_pose@gpose for gpose in self.rel_poses]
    
    def _rotate_around_Z(self, arr: np.ndarray, theta):
        new_arr = arr.copy()

        rot = np.eye(4)
        rot[:2,:2] = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]])
    
        return new_arr@rot
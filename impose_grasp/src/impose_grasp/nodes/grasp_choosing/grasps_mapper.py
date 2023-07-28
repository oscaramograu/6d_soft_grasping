from math import pi, cos, sin
import numpy as np
from typing import List

class GraspMapper:
    def __init__(self, theta: float = 30, offsets: np.ndarray = np.array([0,0,0])):
        self.theta = theta*pi/180
        self.offsets = offsets

        self.obj_pose: np.ndarray

    def map_grasp(self, grasp)->List[np.ndarray]:
        gpose, w = grasp
        mapped_g = self._rotate_around_Z(gpose)
        offseted_g = self._offset(mapped_g)

        return offseted_g, w

    def _rotate_around_Z(self, arr: np.ndarray):
        theta = self.theta
        new_arr = arr.copy()

        rot = np.eye(4)
        rot[:2,:2] = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]])
    
        return new_arr@rot

    def _offset(self, arr: np.ndarray):
        new_arr = arr.copy()
        offsets = self.offsets.copy()

        conv_offsets = new_arr[:3,:3]@offsets
        new_arr[:3,3] += conv_offsets
        
        return new_arr        

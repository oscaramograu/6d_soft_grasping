from math import pi, cos, sin
import numpy as np
from typing import List
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps

class GraspMapper(Grasps):
    def __init__(self, width_proportion_th:float = 0.3,
                theta: float = 30, offsets: np.ndarray = np.array([0,0,0])):
        super().__init__()

        self.theta = theta*pi/180
        self.offsets = offsets
        self.prop_th = width_proportion_th

    def map_grasps(self):
        for ind in range(len(self.rel_poses)):
            grasp = [self.rel_poses[ind], self.widths[ind]]
            self.rel_poses[ind], self.widths[ind] = self._map_grasp(grasp)

    def _map_grasp(self, grasp)->List[np.ndarray]:
        gpose, w = grasp
        mapped_g = self._rotate_around_Z(gpose, self.theta)
        offseted_g = self._offset(mapped_g)

        return offseted_g, w

    def _offset(self, arr: np.ndarray):
        new_arr = arr.copy()
        offsets = self.offsets.copy()

        conv_offsets = new_arr[:3,:3]@offsets
        new_arr[:3,3] += conv_offsets
        
        return new_arr        

    def _with_to_power_g(self, grasp) -> bool:
        max_width = max(self.widths)
        min_widht = min(self.widths)

        width_prop = (grasp[0] - min_widht)/(max_width - min_widht)

        if width_prop > self.prop_th:
            return True
        else:
            return False
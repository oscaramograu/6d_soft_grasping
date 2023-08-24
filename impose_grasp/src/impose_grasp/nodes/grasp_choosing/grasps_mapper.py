from math import pi, cos, sin
import numpy as np
from typing import List
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps

class GraspMapper(Grasps):
    def __init__(self, width_th :float = 0.04,
                theta: float = 30, grasps: Grasps = None):
        super().__init__()

        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self.theta = theta*pi/180
        self.pinch_offsets = np.array([0.015, 0.02, -0.025])
        self.power_offsets = np.array([0.02, 0.01, -0.03])
        self.width_th = width_th
        self.mapped_poses = List[np.ndarray]

    def map_grasps(self):
        for ind in range(len(self.rel_poses)):
            grasp = [self.rel_poses[ind], self.widths[ind]]
            self.rel_poses[ind], power_graps_flag = self._map_grasp(grasp)
            self.power_gr.append(power_graps_flag)

    def _map_grasp(self, grasp)->List[np.ndarray]:
        gpose, w = grasp
        mapped_g = self._rotate_around_Z(gpose, self.theta)
        power_grasp = self._with_to_power_g(w)

        if power_grasp:
            offseted_g = self._pow_offset(mapped_g)
        else:
            offseted_g = self._pinch_offset(mapped_g)

        return offseted_g, power_grasp

    def _pow_offset(self, arr: np.ndarray):
        new_arr = arr.copy()
        offsets = self.power_offsets.copy()

        conv_offsets = new_arr[:3,:3]@offsets
        new_arr[:3,3] += conv_offsets
        
        return new_arr  
          
    def _pinch_offset(self, arr: np.ndarray):
        new_arr = arr.copy()
        offsets = self.pinch_offsets.copy()

        conv_offsets = new_arr[:3,:3]@offsets
        new_arr[:3,3] += conv_offsets
        
        return new_arr        


    def _with_to_power_g(self, width) -> bool:
        if width > self.width_th:
            return True
        else:
            return False
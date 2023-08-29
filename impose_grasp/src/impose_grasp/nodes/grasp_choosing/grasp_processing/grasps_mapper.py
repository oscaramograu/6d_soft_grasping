from math import pi
import numpy as np
from typing import List

from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps

class GraspMapper(GraspsBase):
    def __init__(self, width_th :float = 0.04,
                theta: float = 30, grasps: Grasps = None):
        super().__init__(grasps)

        self.theta = theta*pi/180
        self.pinch_offsets = np.array([0.026, -0.015, -0.025])
        # self.pinch_offsets = np.array([0, 0.01, -0.03])
        self.power_offsets = np.array([0.0, 0.0, -0.02])

        self.pinch_sinergy = [0.7, 0.99]
        self.power_sinergy = [0.99, 0.0]

        self.width_th = width_th

    def map_grasps(self, using_offset = True):
        if self.using_qb_hand:
            for ind in range(len(self.rel_poses)):
                grasp = [self.rel_poses[ind], self.widths[ind]]
                self.rel_poses[ind], sin_values = self._map_grasp(grasp, using_offset)
                self.synergies_values.append(sin_values)
            print("Grasps where mapped")

    def _map_grasp(self, grasp, using_offset = True)->List[np.ndarray]:
        mapped_g, w = grasp
        sin_values = self._compute_sinergies(w, using_th_mode=False)
        if using_offset:
            g_post_off = self._offset(mapped_g, w)
        else:
            g_post_off = mapped_g

        of_and_rot_g = self._rotate_around_X(g_post_off, 10*pi/180)  
        of_and_rot_g = self._rotate_around_Y(of_and_rot_g, -10*pi/180)  
        final_g = self._rotate_around_Z(of_and_rot_g, self.theta)
        return [final_g, sin_values]
    
    def _offset(self, gr_pose: np.ndarray, width):
        new_gr_pose = gr_pose.copy()

        if width > self.width_th:
            offsets = self.pinch_offsets.copy()
        else:
            offsets = self.power_offsets.copy()

        oriented_offsets = new_gr_pose[:3,:3]@offsets
        new_gr_pose[:3,3] += oriented_offsets
        
        return new_gr_pose

    def _compute_sinergies(self, width, using_th_mode = True) -> List[float]:
        if using_th_mode:
            return self._compute_th_sinergies(width)
            
        else:
            return self.pinch_sinergy
        
    def _compute_th_sinergies(self, width):
        if width > self.width_th:
            return self.pinch_sinergy
            
        else:
            return self.power_sinergy
    
    def _compute_lineal_sinergies(self, w): 
        """ I make the assumptiona that minus than 5 mm is a full pinch and more 
        than 5 cm is full grasp."""
        if w < 0.005:
            return self.power_sinergy
        elif w < 0.05:
            singergy_1 = w*25 - 0.25
            singergy_2 = w*-12 + 0.625
            return [singergy_1, singergy_2]
        else:
            return self.pinch_sinergy
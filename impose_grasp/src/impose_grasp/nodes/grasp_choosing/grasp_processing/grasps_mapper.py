from math import pi
import numpy as np
from typing import List

from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps

class GraspMapper(GraspsBase):
    def __init__(self, width_th :float = 0.04,
                theta: float = 30, grasps: Grasps = None):
        super().__init__(grasps)

        self.theta = theta*pi/180
        self.pinch_offsets = np.array([0.01, 0.01, -0.03])
        # self.pinch_offsets = np.array([0, 0, -0.025])
        self.power_offsets = np.array([0.02, 0.01, -0.03])

        self.width_th = width_th

    def map_grasps(self, using_offset = True):
        if self.using_qb_hand:
            for ind in range(len(self.rel_poses)):
                grasp = [self.rel_poses[ind], self.widths[ind]]
                self.rel_poses[ind], power_graps_flag = self._map_grasp(grasp, using_offset)
                self.power_gr_flags.append(power_graps_flag)
            print("Grasps where mapped")

    def _map_grasp(self, grasp, using_offset = True)->List[np.ndarray]:
        gpose, w = grasp
        mapped_g = self._rotate_around_Z(gpose, self.theta)
        using_pw_gr = self._using_pow_gr(w)

        if using_offset:
            offseted_g = self._offset(mapped_g, using_pw_gr)
        else:
            offseted_g = mapped_g
        return offseted_g, using_pw_gr

    def _offset(self, gr_pose: np.ndarray, using_pw_gr):
        new_gr_pose = gr_pose.copy()

        if using_pw_gr:
            new_offsets = self.power_offsets.copy()
        else:
            new_offsets = self.pinch_offsets.copy()

        conv_offsets = new_gr_pose[:3,:3]@new_offsets
        new_gr_pose[:3,3] += conv_offsets
        
        return new_gr_pose  

    def _using_pow_gr(self, width) -> bool:
        if width > self.width_th:
            return True
        else:
            return False
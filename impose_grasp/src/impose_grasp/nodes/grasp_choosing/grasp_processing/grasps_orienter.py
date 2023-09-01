import numpy as np
from math import pi

from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase

class GraspOrienter(GraspsBase):
    def __init__(self, obj_pose) -> None:
        super().__init__()
        self.set_abs_poses(obj_pose)
        self.obj_pose = obj_pose

 
    def orient_grasps(self):
        if self.using_qb_hand:
            obj_to_base_vec = -self.obj_pose[:3,3]/np.linalg.norm(-self.obj_pose[:3,3])
            good_gps_y_inds = self._select_grasp_inds_by_ang(obj_to_base_vec, tr_ang=90, axis=1)
            self._invert_opposite_Ys(good_gps_y_inds)
            print("Grasps where reoriented.")   
        else:
            print("Grasps where not reoriented.")

    def _invert_opposite_Ys(self, good_g_inds):
        """
        If the direction of the Y is pointing oposite to the robot position wrt to the robot,
        the grasp gets rotated 180 degrees.
        """
        inds = range(len(self.rel_poses))
        inverted_gr_y_inds = [x for x in inds if x not in good_g_inds]

        for ind in inverted_gr_y_inds:
            original_pose = self.rel_poses[ind][:3, 3].copy()
            self.rel_poses[ind] = self._rotate_around_Z(self.rel_poses[ind], pi)
            self.rel_poses[ind][:3, 3] = original_pose
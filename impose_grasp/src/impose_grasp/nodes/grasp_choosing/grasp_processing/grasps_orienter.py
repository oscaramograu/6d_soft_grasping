import numpy as np
from math import pi

from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase

class GraspOrienter(GraspsBase):
    def __init__(self, obj_pose) -> None:
        super().__init__()
        self.set_abs_poses(obj_pose)
        self.obj_pose = obj_pose
        self.reoriented = [False for i in self.rel_poses]

 
    def orient_grasps(self):
        if self.using_qb_hand:
            obj_to_base_vec = -self.obj_pose[:3,3]/np.linalg.norm(-self.obj_pose[:3,3])
            obj_base_dist = np.linalg.norm(self.obj_pose[:2, 3])
            if(obj_base_dist > 0.45):
                rotated_obj_base_vec = self._rotate_vect_on_Z(obj_to_base_vec, -30)
                good_gps_y_inds = self._select_grasp_inds_by_ang(rotated_obj_base_vec, tr_ang=90, axis=1)

            # elif(obj_base_dist < 0.40):
            #     rotated_obj_base_vec = self._rotate_vect_on_Z(obj_to_base_vec, 180 - 30)
            #     good_gps_y_inds = self._select_grasp_inds_by_ang(rotated_obj_base_vec, tr_ang=90, axis=1)
            
            else:
                good_gps_y_inds = []

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
        self.reoriented = [x in inverted_gr_y_inds for x in inds]
        
        for ind in inverted_gr_y_inds:
            original_pose = self.rel_poses[ind][:3, 3].copy()
            self.rel_poses[ind] = self._rotate_around_Z(self.rel_poses[ind], pi)
            self.rel_poses[ind][:3, 3] = original_pose
    
    def _rotate_vect_on_Z(self, vect:np.ndarray, ang:float):
        ang_rad = ang/180*np.pi
        rot = np.array([
            [np.cos(ang_rad), -np.sin(ang_rad), 0],
            [np.sin(ang_rad), np.cos(ang_rad), 0],
            [0, 0, 1]])
        return rot@vect
import numpy as  np
from math import pi
from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps

class GraspFilterer(GraspsBase):
    def __init__(self,  grasps: Grasps = None) -> None:
        super().__init__(grasps)
        self._good_grasps_ids = []
        self._bad_grasps_ids = []

    def filter(self, obj_pose: np.ndarray, cam_pose: np.ndarray):
        """
        Given the pose of the object and the camera, the grasps are filtered to consider only
        the ones which's Zs are pointing upwards and Ys are pointing to the robot base. Or the 
        ones that are pointing to the camera frame.
        """
        vertical_vec =  np.array([0,0,-1])

        if self.using_qb_hand:
            pass
            good_grasps_ids  = self._select_grasp_inds_by_ang(vertical_vec, tr_ang=50, axis=2)            
            good_grasps_ids = self._select_higher_grasps(good_grasps_ids, obj_pose, th_dist=0.02)

        else:
            # SELECT ONLY THE ONES THAT POINT TO THE CAMERA OF THE ROBOT
            obj_cam_vec = (obj_pose[:3, 3] - cam_pose[:3, 3])
            obj_cam_vec /= np.linalg.norm(obj_cam_vec)
            good_grasps_ids  = self._select_grasp_inds_by_ang(obj_cam_vec, tr_ang=70, axis=2)

        self._build_good_flags(good_grasps_ids)

        print("The good grasp ids are: ", good_grasps_ids)

    def _select_higher_grasps(self, ids, obj_pose:np.ndarray, th_dist: float = 0):
        """ It selects grasps which are over the target object in the z axis."""
        high_ids = [i for i in ids 
                    if(self.abs_poses[i][2, 3] > obj_pose[2, 3] - th_dist)]
        return high_ids

    def _build_good_flags(self, good_grasps_ids):
        for i in range(len(self.rel_poses)):
            if i in good_grasps_ids:
                self.good_gr_flags.append(True)
            else:
                self.good_gr_flags.append(False)
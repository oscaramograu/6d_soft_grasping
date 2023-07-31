import math
import numpy as np 
import open3d as o3d

from typing import List
from impose_grasp.nodes.grasp_choosing.grasp_chooser_base import GraspChooserBase
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.lib.geometry import invert_homogeneous
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps

class GraspChooser(GraspChooserBase):
    def __init__(self, grasps: Grasps):

        self.cam_pose: np.ndarray  # in global frame

        self.obstruction_pcl: o3d.t.geometry.PointCloud = None # in camera frame
        self.scene: o3d.t.geometry.RaycastingScene 

        super().__init__(grasps)

    def compute_best_grasp_pose(self):
        """
        It selects the best grasping pose for the target 
        object between all the possible candidates:
        - It first removes all grasp pose which its normal 
        has 60 degrees from the camera frame normal.
        - Then it iterates through the selected points.
        - For each grasping position removes all points from 
        the obstruction point cloud which are to far away.
        - Then it calculates the mean distance from each of the 
        obstruction points to the collision mesh of the gripper 
        placed at the current grasping position, to assign a score 
        to that grasping position.
        - Finally based on the score obtained, the best grasping 
        position is retrieved.
        """
        if  self.obstruction_pcl is None:
            print("Obstruction point cloud is none")
            return None

        # find distances to obstruction pointcloud
        best_i = None
        best_score = math.inf

        for i in range(len(self.rel_poses)):
            grasp_depth = self.widths[i]
            gpose = self.rel_poses[i] # From a grasping pose wrt obj

            pcd =  self.obstruction_pcl.clone()
            pcd.transform(self.cam_pose @ invert_homogeneous(gpose))     #NEEDS TO BE REVISED   # get the obstrution points wrt grasping points
            pcd = pcd.select_by_mask(o3d.core.Tensor(
                pcd.point.positions.numpy()[:, 2] < - grasp_depth+0.02))  # dont consider points that "are further away" + finger_over tolerance

            result = self.scene.compute_signed_distance(pcd.point.positions).numpy() # compute the distance of each obstruction point wrt the EEF mesh
                                                                                # placed at distance 
                                # from an obstruction point to the EEF mesh at this p this grasping position
            n_points = np.count_nonzero(result < 0.001)
            if n_points < 1:    # select the grasping point that has the smallest averageoint.
                dist_score = np.mean(1. / result)
                if dist_score < best_score:
                    best_i = i
                    best_score = dist_score

        return self.gripping_poses[best_i]

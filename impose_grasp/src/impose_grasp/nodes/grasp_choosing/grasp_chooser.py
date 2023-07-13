import math
import numpy as np 
import open3d as o3d

from typing import List
from impose_grasp.nodes.grasp_choosing.grasp_chooser_base import GraspChooserBase
from impose_grasp.models.cameras.base_camera import CamFrame
from impose_grasp.lib.geometry import invert_homogeneous

class GraspChooser(GraspChooserBase):
    def __init__(self, target_obj_name):

        self.obj_pose: np.ndarray # in global frame
        self.gripper_pose: np.ndarray  # in global frame
        self.gripping_poses: List[np.ndarray]  # in global frame
        self.grasp_offsets: List[float]  # in m from closed position
        self.obstruction_pcl: o3d.t.geometry.PointCloud = None # in global frame
        self.scene: o3d.t.geometry.RaycastingScene 

        super().__init__(target_obj_name)

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

        inds = range(len(self.gripping_poses))

        # filter gposes "that point away"
        # this avoids grasping the object from "behind"
        poses =  self.gripping_poses
        gripper_z =  self.gripper_pose[:3, 2]

        rel_angle = [np.dot(gripper_z, gpose[:3, 2]) for gpose in poses]
        angle_thr = np.cos(60/180*np.pi)
        # print("The relative angles are:")
        # print("The theshold angle is:", "\n", angle_thr)
        inds = [x for x in inds if rel_angle[x] > angle_thr]

        # find distances to obstruction pointcloud
        best_i = None
        best_score = math.inf

        for i in inds:
            grasp_depth =  self.grasp_offsets[i]
            gpose = poses[i] # From a grasping pose

            pcd =  self.obstruction_pcl.clone()
            pcd.transform(invert_homogeneous(gpose))         # get the obstrution points wrt grasping points
            pcd = pcd.select_by_mask(o3d.core.Tensor(
                pcd.point.positions.numpy()[:, 2] < - grasp_depth+0.02))  # dont consider points that "are further away" + finger_over tolerance

            result = self.scene.compute_signed_distance(pcd.point.positions).numpy() # compute the distance of each obstruction point wrt the EEF mesh
                                                                                # placed at this grasping position
            n_points = np.count_nonzero(result < 0.001)
            if n_points < 1:    # select the grasping point that has the smallest average distance 
                                # from an obstruction point to the EEF mesh at this point.
                dist_score = np.mean(1. / result)
                if dist_score < best_score:
                    best_i = i
                    best_score = dist_score

        return self.gripping_poses[best_i]

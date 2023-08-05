import math
import numpy as np 
import open3d as o3d
import os

from impose_grasp.nodes.visualisation.point_cloud import PointCloud
from impose_grasp.lib.geometry import invert_homogeneous
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh

class GraspChooser(Grasps):
    def __init__(self, grasps: Grasps, obj_name):
        super().__init__()
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self.pcd_builder = PointCloud(obj_name + "_frame")
        self.voxel_size = 0.05

        self.scene: o3d.t.geometry.RaycastingScene 
        self._build_scene()

    def compute_best_grasp_ind(self):
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

        # find distances to obstruction pointcloud
        best_i = None
        best_score = math.inf

        self.pcd_builder.build_pcd_wrt_obj(self.voxel_size)

        for i in range(len(self.rel_poses)):
            grasp_depth = self.widths[i]
            gpose = self.rel_poses[i] # From a grasping pose wrt obj

            pcd = self.pcd_builder.get_pcd()            
            pcd.transform(invert_homogeneous(gpose))     #NEEDS TO BE REVISED   # get the obstrution points wrt grasping points
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

        return best_i
    
    def _build_scene(self):
        """
        Builds the mesh of the movement projection of the end effector.
        """
        EEF_mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models", "gripper_collision_d415.stl")

        gripper_bbox = load_mesh(EEF_mesh_path, tensor=True)
        self.scene = o3d.t.geometry.RaycastingScene()
        _ = self.scene.add_triangles(gripper_bbox)
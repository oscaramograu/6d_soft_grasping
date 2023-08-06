import math
import numpy as np 
import open3d as o3d
import os

from impose_grasp.lib.point_cloud import PointCloud
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh

class GraspChooser(Grasps):
    def __init__(self, obj_name, grasps: Grasps):
        super().__init__()
        self.set_rel_poses(grasps.rel_poses)
        self.set_widths(grasps.widths)
        self.set_power_gr(grasps.power_gr)

        self.pcd_builder = PointCloud(obj_name + "_frame")
        self.voxel_size = 0.00

        self.scene: o3d.t.geometry.RaycastingScene 
        self._build_scene()

    def compute_best_grasp_ind(self):
        """
        It selects the best grasping pose for the target 
        object between all the possible registered candidates.

        For each grasp candidate:
        - It  selects only points that are at less than 15 cm from the 
        grasp candidate position. 
        - It computes the number of ostructioin pts inside the collisioin
        mesh palced at the grasp candidate position.
        - If no points are inside the collision mesh, calculates the mean 
        distance from each of the obstruction points to the collision mesh
        to assign a score to that grasp candidate.
        - If all candidates have points inside its collision mesh, the one
        with less pts is selected.
        """
        best_i = None
        best_score = math.inf
        self.pcd_builder.set_new_pcd_wrt_obj(self.voxel_size)
        pts_in_col = []

        for i in range(len(self.rel_poses)):
            gpose = self.rel_poses[i] # From a grasping pose wrt obj

            pcd = self.pcd_builder.get_pcd_wrt_target(gpose)
            pcd = self.pcd_builder.select_pts_in_range(pcd, 0.15)
            result = self.scene.compute_signed_distance(pcd.point.positions).numpy()
            n_points = np.count_nonzero(result < -0.01)

            if n_points < 1:  
                print("The grasp: ", i, ", has no points in its grasping volume.")

                dist_score = np.mean(1. / result)
                if dist_score < best_score:
                    best_i = i
                    best_score = dist_score

            pts_in_col.append(n_points)
            
        if best_i == None:
            best_i = pts_in_col.index(min(pts_in_col))

        return best_i
    
    def _build_scene(self):
        """
        Builds the mesh of the movement projection of the end effector.
        """
        EEF_mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models", "hand_col.stl")

        gripper_bbox = load_mesh(EEF_mesh_path, tensor=True)
        self.scene = o3d.t.geometry.RaycastingScene()
        _ = self.scene.add_triangles(gripper_bbox)
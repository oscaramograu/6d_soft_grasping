import math
import numpy as np 
import open3d as o3d
import os

from impose_grasp.lib.point_cloud import PointCloud
from impose_grasp.nodes.grasp_choosing.grasps_base import GraspsBase, Grasps
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP, load_mesh

class GraspChooser(GraspsBase):
    def __init__(self, grasps: Grasps):
        super().__init__(grasps)

        self.pcd_builder = PointCloud(self.obj_name + "_frame")
        self.voxel_size = 0.00

        self.scene: o3d.t.geometry.RaycastingScene 
        self._build_eef()

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
        dist_scores = []

        good_grasp_ids = [i for i in range(len(self.rel_poses)) 
                          if self.good_gr_flags[i]]  

        for i in good_grasp_ids:
            gpose = self.rel_poses[i] # From a grasping pose wrt obj
            pcd = self.pcd_builder.get_pcd_wrt_target(gpose)
            pcd = self.pcd_builder.select_pts_in_range(pcd, 0.30)
            result = self.scene.compute_signed_distance(pcd.point.positions).numpy()

            if self.using_qb_hand: th = -0.00
            else: th = 0.1

            n_points = np.count_nonzero(result < th)

            if n_points < 1:  
                print("The grasp: ", i, ", has no points in its grasping volume.")
                dist_score = np.mean(1. / result)
                dist_scores.append(dist_score)
                if dist_score < best_score:
                    best_i = i
                    best_score = dist_score
            else:
                dist_scores.append(None)

            pts_in_col.append(n_points)
            
        if best_i == None and self.using_qb_hand:
            index = pts_in_col.index(min(pts_in_col))
            best_i = good_grasp_ids[index]
            print("There are some points in collision with the target grasp!")
        elif best_i == None and not self.using_qb_hand:
            print("No collision free grasp was found.")
        
        print("List of good grasps ids: ", good_grasp_ids)
        print("List of pointcloud pts for each valid grasp: ", pts_in_col)
        print("List of socres for each valid grasp: ", dist_scores)
        print("The grasp sinergy values are: ", self.synergies_values[best_i])
        print("The widht is: ", self.widths[best_i])
        return best_i
    
    def _build_eef(self):
        """
        Builds the mesh of the movement projection of the end effector.
        """
        if self.using_qb_hand: eef = "qb_hand_col_pinch"
        else: eef = "qb_gripper_col"

        EEF_mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models", eef + ".stl")

        gripper_bbox = load_mesh(EEF_mesh_path, tensor=True)
        self.scene = o3d.t.geometry.RaycastingScene()
        _ = self.scene.add_triangles(gripper_bbox)
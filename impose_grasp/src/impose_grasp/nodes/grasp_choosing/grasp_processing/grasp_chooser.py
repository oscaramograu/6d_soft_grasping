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

        self.eef_scene: o3d.t.geometry.RaycastingScene 
        self.fingers_scene: o3d.t.geometry.RaycastingScene 

        self._build_eef()
        self._build_fingers()

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
        self.pcd_builder.set_new_pcd_wrt_obj(self.voxel_size)

        good_grasp_ids = [i for i in range(len(self.rel_poses)) 
                          if self.good_gr_flags[i]]  

        self._choose_best_id(good_grasp_ids)
        
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
        self.eef_scene = o3d.t.geometry.RaycastingScene()
        _ = self.eef_scene.add_triangles(gripper_bbox)

    def _build_fingers(self):
        """
        Builds a scene which consists on the fingers positions for a closed grasp. 
        This will be used to select the grasp which has less points in collision with 
        the fingers.
        """
        eef = "qb_hand_col_fingers"

        EEF_mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models", eef + ".stl")

        fingers_bbox = load_mesh(EEF_mesh_path, tensor=True)
        self.fingers_scene = o3d.t.geometry.RaycastingScene()
        _ = self.fingers_scene.add_triangles(fingers_bbox)

    def _check_points_in_scene(self, scene, pose):
        pcd = self.pcd_builder.get_pcd_wrt_target(pose)
        pcd = self.pcd_builder.select_pts_in_range(pcd, range=0.30)
        result = scene.compute_signed_distance(pcd.point.positions).numpy()

        if self.using_qb_hand: th = -0.00
        else: th = 0.1

        n_points = np.count_nonzero(result < th)

        if n_points < 0:
            dist_score = np.mean(1. / result)
        else:
            dist_score = 0
        return n_points, dist_score    

    def _best_ids_in_scene(self, good_grasp_ids, scene):
        n_eef_pts = []
        eef_avg_ds = []

        for i in good_grasp_ids:
            gpose = self.rel_poses[i]
            n_eef_pt, eef_avg_d = self._check_points_in_scene(scene, gpose)
            n_eef_pts.append(n_eef_pt)
            eef_avg_ds.append(eef_avg_d)

        min_eef_pts = [n_eef_pts.index(min(n_eef_pts)), min(n_eef_pts)]
        max_eef_avg_ds = [eef_avg_ds.index(max(eef_avg_ds)), max(eef_avg_ds)]

        return min_eef_pts, max_eef_avg_ds
    
    def _choose_best_id(self, good_g_ids):
        min_eef_pts, max_eef_dist = self._best_ids_in_scene(good_g_ids, self.eef_scene)

        if min_eef_pts[1] == 0:
            if self.using_qb_hand:
                min_fing_pts, max_fing_dist = self._best_ids_in_scene(good_g_ids, self.eef_scene)
                if min_fing_pts[1] == 0:
                    best_i = max_fing_dist[0]
                else:
                    best_i = min_fing_pts[0]
            else:
                best_i = max_eef_dist[0]
        else:
            best_i = None
        
        return best_i
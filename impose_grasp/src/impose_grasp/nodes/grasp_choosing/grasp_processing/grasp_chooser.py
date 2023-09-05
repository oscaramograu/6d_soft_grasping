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

        self.min_eef_pts = {}
        self.min_avg_ds = {}

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
        self.pcd_builder.set_new_pcd_wrt_obj(self.voxel_size)

        good_grasp_ids = [i for i in range(len(self.rel_poses)) 
                          if self.good_gr_flags[i]]  
        
        return self._choose_best_id(good_grasp_ids)
    
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
        eef = "qb_hand_col_pinch_fingers"

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

        if n_points == 0:
            dist_score = np.mean(1. / result)
        else:
            dist_score = math.inf
        return n_points, dist_score    

    def _build_targ_dict(self, id, n_eef_pt, eef_avg_d):
        targ_dic = {
            "id": id,
            "num_eef_pts": n_eef_pt,
            "eef_avg_dist": eef_avg_d
        }
        return targ_dic

    def _best_ids_in_scene(self, good_grasp_ids, scene):
        n_eef_pts = []
        eef_avg_ds = []

        for i in good_grasp_ids:
            gpose = self.rel_poses[i]
            n_eef_pt, eef_avg_d = self._check_points_in_scene(scene, gpose)
            n_eef_pts.append(n_eef_pt)
            eef_avg_ds.append(eef_avg_d)

        print("The good grasps are: ", good_grasp_ids)
        print("The number of collision pts for each grasps are: ", n_eef_pts)
        print("The average distances to the collision meshes are: ", eef_avg_ds)

        min_pts_id = n_eef_pts.index(min(n_eef_pts))
        min_eef_pts = self._build_targ_dict(good_grasp_ids[min_pts_id], min(n_eef_pts), eef_avg_ds[min_pts_id])

        min_score_id = eef_avg_ds.index(min(eef_avg_ds))
        min_avg_ds = self._build_targ_dict(good_grasp_ids[min_score_id], n_eef_pts[min_score_id], min(eef_avg_ds))

        return min_eef_pts, min_avg_ds
    
    def _choose_best_id(self, good_g_ids):
        self.min_eef_pts, self.min_avg_ds = self._best_ids_in_scene(good_g_ids, self.eef_scene)

        if self.min_eef_pts["num_eef_pts"] == 0:
            # if self.using_qb_hand:
            #     min_fing_pts, max_fing_dist = self._best_ids_in_scene(good_g_ids, self.eef_scene)
            #     if min_fing_pts[1] == 0:
            #         best_i = max_fing_dist[0]
            #     else:
            #         best_i = min_fing_pts[0]
            # else:
                best_i = self.min_avg_ds["id"]
        else:
            best_i = self.min_eef_pts["id"]

            print("All the grasps have points in collision with the EEF.")
        
        return best_i
    
    def get_chosen_crit(self, best_id):
        if  self.min_eef_pts["id"] == best_id:
            return self.min_eef_pts["num_eef_pts"], self.min_eef_pts["eef_avg_dist"]
        else:
            print("ksdkf")
            return self.min_avg_ds["num_eef_pts"], self.min_avg_ds["eef_avg_dist"]
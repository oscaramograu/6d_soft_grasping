import os
import json
import numpy as  np
from math import cos, sin, pi
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

class GraspFilterer():
    def __init__(self, obj) -> None:
        grasps_path = os.path.join(
            PATH_TO_IMPOSE_GRASP, "data", "models", obj,"gripping_poses.json")
        self._good_grasps = []
        self._bad_grasps = []

        self.grasp_offsets = []
        self.gripping_poses = []
        self.abs_poses = []

        self._load_values(grasps_path)

    def _load_values(self, path):
        if os.path.isfile(path):
            with open(path) as F:
                json_load = json.load(F)
            for x in json_load:     
                self.gripping_poses.append(eval('np.array(' + x["pose"] + ')'))
                self.grasp_offsets.append(x["width"])
        else:
            print("no file found")
        
    def _convert_to_absolute_coords(self, object_pose: np.ndarray):
        for gpose in self.gripping_poses:
            self.abs_poses.append(object_pose@gpose)

    def filter(self, camera_pose: np.ndarray, obj_pose: np.ndarray):
        """
        Given the pose of the object and the camera, the grasps are filtered to consider only
        the ones which's Zs are pointing upwards and Ys are pointing to the robot base.
        """
        self._convert_to_absolute_coords(obj_pose)

        obj_to_base = -obj_pose[:3,3]/np.linalg.norm(-obj_pose[:3,3])
        good_gps_y_inds = self._select_inds_by_rel_ang(obj_to_base, tr_ang=110, axis=1)
        self._invert_opposite(good_gps_y_inds)

        z_vec =  np.array([0,0,-1])
        self._good_grasps  = self._select_inds_by_rel_ang(z_vec, tr_ang=70, axis=2)

        inds = range(len(self.abs_poses))
        self._bad_grasps = [x for x in inds if x not in self._good_grasps]
        print("The number of filtered grasps is: ", len(self._good_grasps))

    def _invert_opposite(self, good_g_inds):
        """
        If the direction of the Y is pointing oposite to the robot position wrt to the robot,
        the grasp gets rotated 180 degrees.
        """
        inds = range(len(self.abs_poses))
        inverted_g_y_inds = [x for x in inds if x not in good_g_inds]

        for ind in inverted_g_y_inds:
            self.gripping_poses[ind] = self._rotate_pi_Z(self.gripping_poses[ind])

    def _rotate_pi_Z(self, arr: np.ndarray):
        theta = pi
        new_arr = arr.copy()

        rot = np.eye(4)
        rot[:2,:2] = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]])
    
        return new_arr@rot
       
    def _select_inds_by_rel_ang(self, vect:np.ndarray, tr_ang: float, axis: int):
        """
        It filters the absolute pose grasps to select only the ones which's selected axis
        angle wrt the given vector is smaller than the given threshold angle.

        Keyword arguments:
        axis -- from 0 to 2 are the x to z respectiveley
        tr_ang -- in degrees
        """
        gposes = self.abs_poses
        rel_ang = [np.dot(vect, gpose[:3, axis]) for gpose in gposes]
        angle_thr = np.cos(tr_ang/180*np.pi)
        inds = range(len(gposes))

        return [x for x in inds if (rel_ang[x] > angle_thr)]

    def get_good_grasps(self):
        good_grasps = [[self.gripping_poses[x], self.grasp_offsets[x]]
            for x in self._good_grasps]
        
        return good_grasps

    def get_bad_grasps(self):
        bad_grasps = [[self.gripping_poses[x], self.grasp_offsets[x]]
            for x in self._bad_grasps]
        
        return bad_grasps
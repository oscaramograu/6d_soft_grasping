import os
import json
import numpy as  np

from sklearn.preprocessing import normalize
from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

class GraspFilterer():
    def __init__(self, obj, obj_pose: np.ndarray, cam_pose: np.ndarray) -> None:
        grasps_path = os.path.join(
            PATH_TO_IMPOSE_GRASP, "data", "models", obj,"gripping_poses.json")
        self._good_grasps = []
        self._bad_grasps = []

        self.grasp_offsets = []
        self.gripping_poses = []
        self._load_values(grasps_path)
        self._convert_to_absolute_coords(obj_pose)
        self._filter(cam_pose)

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
        n=0

        for gpose in self.gripping_poses:
            self.gripping_poses[n] = object_pose@gpose
            n+=1

    def _filter(self, camera_pose: np.ndarray):
        poses =  self.gripping_poses
        camera_z =  camera_pose[:3, 2]

        rel_angle = [np.dot(camera_z, gpose[:3, 2]) for gpose in poses]
        angle_thr = np.cos(60/180*np.pi)

        inds = range(len(self.gripping_poses))
        self._good_grasps = [x for x in inds if rel_angle[x] > angle_thr]
        self._bad_grasps = [x for x in inds if x not in self._good_grasps]

    def get_good_grasps(self):
        good_grasps = [[self.gripping_poses[x], self.grasp_offsets[x]]
            for x in self._good_grasps]
        
        return good_grasps

    def get_bad_grasps(self):
        bad_grasps = [[self.gripping_poses[x], self.grasp_offsets[x]]
            for x in self._bad_grasps]
        
        return bad_grasps
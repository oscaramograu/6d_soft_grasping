import os
import json
import numpy as np
from dataclasses import dataclass
import rospy
from typing import List
from math import pi, cos, sin

from impose_grasp.lib.utils import PATH_TO_IMPOSE_GRASP

@dataclass
class Grasps:
    rel_poses: List[np.ndarray]
    widths: List [float]
    abs_poses: List[np.ndarray]
    good_gr_flags: List [bool]
    synergies_values: List [List [float]]
    reoriented: List[bool]

class GraspsBase(Grasps):
    obj_name: str
    using_qb_hand: bool

    def __init__(self, grasps: Grasps = None) -> None:
        self._load_ros_params()
        self._load_grasp_params(grasps)

    def _load_ros_params(self):
        self.obj_name = rospy.get_param("/target_object")
        eef = rospy.get_param("/robot_config")
        if eef == "qb_hand":
            self.using_qb_hand = True
        elif eef  == "gripper":
            self.using_qb_hand = False

    def _load_grasp_params(self, grasps: Grasps):
        if grasps == None:
            self._load_from_file()
        else:
            super().__init__(
                grasps.rel_poses,
                grasps.widths,
                grasps.abs_poses,
                grasps.good_gr_flags,
                grasps.synergies_values,
                grasps.reoriented)

    def _load_from_file(self):
        MODELS_PATH = os.path.join(PATH_TO_IMPOSE_GRASP, "data", "models")
        grasps_path = os.path.join(MODELS_PATH, self.obj_name, "gripping_poses.json")

        rel_poses = []
        widths = []
        if os.path.isfile(grasps_path):
            with open(grasps_path) as F:
                json_load = json.load(F)
            for x in json_load:     
                rel_poses.append(eval('np.array(' + x["pose"] + ')'))
                widths.append(x["width"])
        super().__init__(
            rel_poses,
            widths,
            [], [], [], [])
    
    def set_abs_poses(self, obj_pose:np.ndarray):
        self.abs_poses = [obj_pose@gpose for gpose in self.rel_poses]
    
    def _rotate_around_Z(self, arr: np.ndarray, theta):
        new_arr = arr.copy()
        Ri = new_arr[:3, :3]

        rot = np.eye(3)
        rot[:2,:2] = np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]])
        Rf = Ri@rot
        new_arr[:3, :3] = Rf
        return new_arr
    
    def _rotate_around_Y(self, arr: np.ndarray, theta):
        new_arr = arr.copy()
        R_i =new_arr[:3, :3]
        rot = np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]
        ]) 
        R_f = R_i@rot
        new_arr[:3, :3] = R_f

        return new_arr
    
    def _rotate_around_X(self, arr: np.ndarray, theta):
        new_arr = arr.copy()
        R_i =new_arr[:3, :3]
        rot = np.array([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)]
        ]) 
        R_f = R_i@rot
        new_arr[:3, :3] = R_f

        return new_arr
    
    def _select_grasp_inds_by_ang(self, vect:np.ndarray, tr_ang: float, axis: int):
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
        selected_ids = [x for x in inds if (rel_ang[x] > angle_thr)]
        return selected_ids
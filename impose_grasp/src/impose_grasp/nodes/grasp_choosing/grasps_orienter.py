import numpy as np
from math import pi
import rospy

from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps
from impose_grasp.lib.tf_listener import TfListener

class GraspOrienter(Grasps):
    def __init__(self) -> None:
        super().__init__()
        obj_name = rospy.get_param("/target_object")
        self.pose_listener = TfListener(obj_name + "_frame")
        self.obj_pose = self._compute_obj_pose()

        eef = rospy.get_param("/robot_config")
        if eef == "qb_hand":
            self.qb_flag = True
        elif eef  == "gripper":
            self.qb_flag = False

    def _compute_obj_pose(self):
        self.pose_listener.listen_tf()
        return self.pose_listener.get_np_frame()

    def get_obj_pose(self):
        return self.obj_pose
    
    def orient_grasps(self):
        self.obj_pose = self._compute_obj_pose()
        self.set_abs_poses(self.obj_pose)

        if self.qb_flag:
            obj_to_base_vec = -self.obj_pose[:3,3]/np.linalg.norm(-self.obj_pose[:3,3])
            good_gps_y_inds = self._select_grasp_inds_by_ang(obj_to_base_vec, tr_ang=90, axis=1)
            self._invert_opposite_Ys(good_gps_y_inds)
            print("Grasps where reoriented.")
        else:
            print("Grasps where not reoriented.")

    def _invert_opposite_Ys(self, good_g_inds):
        """
        If the direction of the Y is pointing oposite to the robot position wrt to the robot,
        the grasp gets rotated 180 degrees.
        """
        inds = range(len(self.rel_poses))
        inverted_gr_y_inds = [x for x in inds if x not in good_g_inds]

        for ind in inverted_gr_y_inds:
            original_pose = self.rel_poses[ind][:3, 3].copy()
            self.rel_poses[ind] = self._rotate_around_Z(self.rel_poses[ind], pi)
            self.rel_poses[ind][:3, 3] = original_pose

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

        return [x for x in inds if (rel_ang[x] > angle_thr)]
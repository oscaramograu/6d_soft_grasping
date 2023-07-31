import numpy as  np
from math import pi
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps
from impose_grasp.lib.tf_listener import TfListener

class GraspFilterer(Grasps):
    def __init__(self, grasps: Grasps = None, obj_name = "cpsduck") -> None:
        super().__init__()

        self._good_grasps_ids = []
        self._bad_grasps_ids = []

        self._tf_listener = TfListener("/" + obj_name + "_frame")

        if grasps == None:
            self.load_from_file(obj_name)
        else:
            self.set_rel_poses(grasps.rel_poses)
            self.set_widths(grasps.widths)
            self.set_power_gr(grasps.power_gr)

    def filter(self):
        """
        Given the pose of the object and the camera, the grasps are filtered to consider only
        the ones which's Zs are pointing upwards and Ys are pointing to the robot base.
        """
        obj_pose = self._listen_obj_pose()
        self.set_abs_poses(obj_pose)

        obj_to_base = -obj_pose[:3,3]/np.linalg.norm(-obj_pose[:3,3])
        good_gps_y_inds = self._select_grasp_inds_by_ang(obj_to_base, tr_ang=110, axis=1)
        self._invert_opposite_Ys(good_gps_y_inds)

        z_vec =  np.array([0,0,-1])
        self._good_grasps_ids  = self._select_grasp_inds_by_ang(z_vec, tr_ang=70, axis=2)

        inds = range(len(self.abs_poses))
        self._bad_grasps = [x for x in inds if x not in self._good_grasps_ids]
        print("The number of filtered grasps is: ", len(self._good_grasps_ids))

    def _listen_obj_pose(self)-> np.ndarray:
        self._tf_listener.listen_tf()
        obj_pose = self._tf_listener.get_np_frame()
        return obj_pose
    
    def _invert_opposite_Ys(self, good_g_inds):
        """
        If the direction of the Y is pointing oposite to the robot position wrt to the robot,
        the grasp gets rotated 180 degrees.
        """
        inds = range(len(self.abs_poses))
        inverted_g_y_inds = [x for x in inds if x not in good_g_inds]

        for ind in inverted_g_y_inds:
            self.rel_poses[ind] = self._rotate_around_Z(self.rel_poses[ind], pi)
       
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

    def get_good_grasps(self):
        poses = []
        widths = []
        power_gr = []

        for i in self._good_grasps_ids:
            poses.append(self.rel_poses[i])
            widths.append(self.widths[i])
            power_gr.append(self.power_gr)

        good_grasps = Grasps()
        good_grasps.set_rel_poses(poses)
        good_grasps.set_widths(widths)
        good_grasps.set_power_gr(power_gr)
        
        return good_grasps

    def get_bad_grasps(self):
        poses = []
        widths = []

        for i in self._bad_grasps_ids:
            poses.append(self.rel_poses[i])
            widths.append(self.widths[i])
            
        bad_grasps = Grasps
        bad_grasps.set_rel_poses(poses)
        bad_grasps.set_widths(widths)

        return bad_grasps
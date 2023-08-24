import numpy as  np
from math import pi
import rospy
from impose_grasp.nodes.grasp_choosing.grasps_base import Grasps
from impose_grasp.lib.tf_listener import TfListener

class GraspFilterer(Grasps):
    def __init__(self, obj_pose,  grasps: Grasps = None) -> None:
        super().__init__()

        self.robot_config = rospy.get_param("/robot_config")
        obj_name = rospy.get_param("/target_object")
        self._good_grasps_ids = []
        self._bad_grasps_ids = []

        # self._obj_tf_listener = TfListener(obj_name + "_frame")
        self._cam_tf_listener = TfListener(target_frame="camera_frame")
        self._obj_pose = obj_pose

        if grasps == None:
            self.load_from_file(obj_name)
            self.set_power_gr([1 for _ in range(len(self.widths))])
        else:
            self.set_rel_poses(grasps.rel_poses)
            self.set_widths(grasps.widths)
            self.set_power_gr(grasps.power_gr)

    def filter(self):
        """
        Given the pose of the object and the camera, the grasps are filtered to consider only
        the ones which's Zs are pointing upwards and Ys are pointing to the robot base. Or the 
        ones that are pointing to the camera frame.
        """
        # obj_pose, cam_pose = self._listen_poses()
        cam_pose = self._listen_poses()
        obj_pose = self._obj_pose

        self.set_abs_poses(obj_pose)
        vertical_vec =  np.array([0,0,-1])

        if cam_pose is None:
            # obj_to_base_vec = -obj_pose[:3,3]/np.linalg.norm(-obj_pose[:3,3])
            # good_gps_y_inds = self._select_grasp_inds_by_ang(obj_to_base_vec, tr_ang=90, axis=1)
            # self._invert_opposite_Ys(good_gps_y_inds)

            good_grasps_ids  = self._select_grasp_inds_by_ang(vertical_vec, tr_ang=40, axis=2)
            good_grasps_ids = self._select_only_positive_points(good_grasps_ids, obj_pose, th_dist=0.02)

        elif cam_pose is not None:
            # SELECT ONLY THE ONES THAT POINT TO THE CAMERA OR THE ROBOT
            obj_cam_vec = (obj_pose[:3, 3] - cam_pose[:3, 3])
            obj_cam_vec /= np.linalg.norm(obj_cam_vec)
            good_grasps_ids  = self._select_grasp_inds_by_ang(obj_cam_vec, tr_ang=70, axis=2)

        # self._good_grasps_ids = self._exclude_lower_grasps(good_grasps_ids, obj_pose)
        self._good_grasps_ids = good_grasps_ids
        inds = range(len(self.abs_poses))
        self._bad_grasps = [x for x in inds if x not in self._good_grasps_ids]
        print("The number of filtered grasps is: ", len(self._good_grasps_ids))

    def _exclude_lower_grasps(self, good_g_inds, obj_pose):
        z_th =  obj_pose[2,3] + 0.01     
        new_inds = [ind for ind in good_g_inds if 
                    self.abs_poses[ind][2,3] > z_th]
        return new_inds

    def _listen_poses(self)-> (np.ndarray, np.ndarray):
        # self._obj_tf_listener.listen_tf()
        # obj_pose = self._obj_tf_listener.get_np_frame()

        if self.robot_config == "gripper":
            self._cam_tf_listener.listen_tf()
            cam_pose = self._cam_tf_listener.get_np_frame()
        else:
            cam_pose = None
        # return obj_pose, cam_pose
        return cam_pose

    
    def _invert_opposite_Ys(self, good_g_inds):
        """
        If the direction of the Y is pointing oposite to the robot position wrt to the robot,
        the grasp gets rotated 180 degrees.
        """
        inds = range(len(self.abs_poses))
        inverted_gr_y_inds = [x for x in inds if x not in good_g_inds]

        for ind in inverted_gr_y_inds:
            original_pose = self.rel_poses[ind][:3, 3].copy()
            self.rel_poses[ind] = self._rotate_around_Z(self.rel_poses[ind], pi)
            self.rel_poses[ind][:3, 3] = original_pose

    def _select_only_positive_points(self, ids, obj_pose:np.ndarray, th_dist: float = 0):
        """ It selects grasps which are over the target object in the z axis."""
        high_ids = [i for i in ids if(self.abs_poses[i][2, 3] > obj_pose[2, 3] - th_dist)]
        # for i in high_ids:
        #     z = self.abs_poses[i][2, 3]
        #     print(z)
        # print("The abs pose of z is: ", obj_pose[2, 3])
        return high_ids



    def get_good_grasps(self):
        poses = []
        widths = []
        power_gr = []

        for i in self._good_grasps_ids:
            poses.append(self.rel_poses[i])
            widths.append(self.widths[i])
            power_gr.append(self.power_gr[i])

        good_grasps = Grasps()
        good_grasps.set_rel_poses(poses)
        good_grasps.set_widths(widths)
        good_grasps.set_power_gr(power_gr)
    
        return good_grasps, self._good_grasps_ids

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
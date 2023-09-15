from typing import List
import rospy
import numpy as np
import os
import open3d as o3d
import math
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import MarkerArray, Marker

from impose_grasp.nodes.grasp_choosing.grasp_for_collision.mesh_projection import MeshUtils
from impose_grasp.lib.geometry_utils import VectorOperator
from impose_grasp.lib.tf_listener import TfListener
from impose_grasp.lib.tf_broadcaster import TransformBroadcaster
from impose_grasp.lib.utils import load_mesh, PATH_TO_IMPOSE_GRASP
from impose_grasp.lib.point_cloud import PointCloud
from impose_grasp.lib.geometry import invert_homogeneous
from impose_grasp.lib.np_angle_calc import rotate_x

class EEFScene():
    eef_scene: o3d.t.geometry.RaycastingScene 

    def __init__(self, eef = "qb_hand_col_pinch"):
        """
        Builds the mesh of the movement projection of the end effector.
        """
        EEF_mesh_path = os.path.join(PATH_TO_IMPOSE_GRASP,
            "data", "models", eef + ".stl")

        gripper_bbox = load_mesh(EEF_mesh_path, tensor=True)
        self.eef_scene = o3d.t.geometry.RaycastingScene()
        _ = self.eef_scene.add_triangles(gripper_bbox)
    
    def compute_collision_pts_and_score(self, pose, point_cld: PointCloud):
        pcd = point_cld.get_pcd_wrt_target(pose)
        pcd = point_cld.select_pts_in_range(pcd, range=0.30)
        result = self.eef_scene.compute_signed_distance(pcd.point.positions).numpy()

        th = 0.0

        n_points = np.count_nonzero(result < th)

        if n_points == 0:
            dist_score = np.mean(1. / result)
        else:
            dist_score = math.inf
        return n_points, dist_score
    
class MeshGraspSelector(MeshUtils):
    grasp_candidates_ids: List[np.ndarray]
    obj_pose: np.ndarray
    cam_pose: np.ndarray

    obj_pose_listener: TfListener
    cam_pose_lilstener: TfListener

    tr_ang: float
    operator: VectorOperator
    eef_scene: EEFScene
    col_pcd: PointCloud

    trans_msh_pts: np.ndarray

    sel_npts: float
    best_score: float
    sel_pose: float
    grasp_candidates_poses: List[np.array]

    def __init__(self, object_name, tr_ang=10) -> None:
        self.load_mesh(object_name)
        self.obj_pose_listener = TfListener(object_name + "_frame")
        self.cam_pose_lilstener = TfListener("camera_frame")
        self.tr_ang = tr_ang*np.pi/180
        self.operator = VectorOperator()
        self.grasp_candidates_ids = []
        self.grasp_candidates_poses = []

        self.col_pcd = PointCloud(object_name + "_frame")
        self.eef_scene = EEFScene()

        self.sel_npts = math.inf
        self.best_score = math.inf
        self.sel_pose = None

    def select_grasp(self):
        self.set_obj_and_cam_poses()
        self.select_cands_by_ang()
        self.col_pcd.set_new_pcd_wrt_obj(voxel_size=0)
        i = 0
        max_id, min_id = self.find_edges_ids()

        for id in self.grasp_candidates_ids:
            pt = self.trans_msh_pts[id]
            pose1, pose2 = self.build_random_oriented_poses(pt, max_id) # wrt base

            self.find_collisions_in_pose(pose1, id, i)
            self.find_collisions_in_pose(pose2, id, i)
            i+=1

        # print("The selected pose is at: ", self.sel_pose)
        # print("The score is :", self.best_score)
        # print("The n of pts is :", self.sel_npts)

        return self.sel_pose
    
    def find_collisions_in_pose(self, pose, id, i):
            pose_wrt_obj = pose.copy()@invert_homogeneous(self.obj_pose)
            pose_wrt_obj[:3, 3] = self.mesh_pts[id]

            self.grasp_candidates_poses.append(pose_wrt_obj)
            n_pts, score = self.eef_scene.compute_collision_pts_and_score(pose_wrt_obj, self.col_pcd)
            # print(i, ": ", score)
            if n_pts == 0 and score < self.best_score:
                self.best_score = score
                self.sel_pose = pose_wrt_obj
                self.sel_npts = n_pts

    def set_obj_and_cam_poses(self):
        self.cam_pose_lilstener.listen()
        self.obj_pose_listener.listen()

        self.cam_pose = self.cam_pose_lilstener.get_np_frame()
        self.obj_pose = self.obj_pose_listener.get_np_frame()
    
    def select_cands_by_ang(self):
        self.trans_msh_pts = self.transform_mesh(self.obj_pose)
        obj_cam_vec = self.cam_pose[:3, 3] - self.obj_pose[:3, 3]
        # obj_cam_vec_r = rotate_x(obj_cam_vec.copy(), 20)

        min_ang = math.inf
        for i in range(self.trans_msh_pts.shape[0]):
            pt = self.trans_msh_pts[i]
            obj_pt_vec = pt - self.obj_pose[:3, 3].T
            angle = self.operator.compute_angle(init_vect=obj_cam_vec, end_vect=obj_pt_vec)
            if angle < self.tr_ang:
                if angle < min_ang: min_ang = angle
                self.grasp_candidates_ids.append(i)
        # print(min_ang*180/math.pi)

    def build_random_oriented_poses(self, point:np.ndarray, max_id):
        Rt_pose1 = np.eye(4)
        Rt_pose1[:3, 3] = point.copy()
        # Rt_pose2 = Rt_pose1.copy()

        point_obj_vec = self.obj_pose[:3, 3].T - point.copy()
        z_axis = point_obj_vec/np.linalg.norm(point_obj_vec)
        normal_Z_plane = self.operator.perpendicular_plane(point_obj_vec)

        max_edge_vect = self.trans_msh_pts[max_id] - self.obj_pose[:3, 3]
        x_axis1 = self.operator.project_vector_onto_plane(max_edge_vect, normal_Z_plane)
        x_axis1 /= np.linalg.norm(max_edge_vect)
        # x_axis2 = -max_edge_vect/max_edge_vect/np.linalg.norm(max_edge_vect)

        y_axis1 = np.cross(z_axis, x_axis1)
        # y_axis2 = np.cross(z_axis, x_axis2)

        # normal_Z_plane = self.operator.perpendicular_plane(point_obj_vec)
        # point_robot_vec = -point.copy()
        # pt_robot_z_axis_n_vec = np.cross(np.array([0, 0, 1]), point_robot_vec)
        # pt_robot_z_axis_plane = self.operator.perpendicular_plane(pt_robot_z_axis_n_vec)
        # y_axis = -self.operator.intersection_vector(normal_Z_plane, pt_robot_z_axis_plane)
        # y_axis /= np.linalg.norm(y_axis)

        # x_axis = np.cross(y_axis, z_axis)
        Rt_pose1[:3, :3] = np.column_stack((x_axis1, y_axis1, z_axis))
        # Rt_pose2[:3, :3] = np.column_stack((x_axis2, y_axis2, z_axis))

        rand_ang = np.random.random()*20
        if np.random.random() > 0.5: rand_ang *= -1   

        Rt_pose1 = self.operator.rotate_Rt_on_z(Rt_pose1, - 30)
        Rt_pose1 = self.operator.rotate_Rt_on_z(Rt_pose1, rand_ang)
        Rt_pose2 = self.operator.rotate_Rt_on_z(Rt_pose1.copy(), 180)

        return Rt_pose1, Rt_pose2
    
class MarkerArrayBuilder:
    array: MarkerArray
    pub: rospy.Publisher

    def __init__(self) -> None:
        self.array = MarkerArray()
        self.pub = rospy.Publisher("marker_array", MarkerArray, queue_size=10)

    def build_array_from_mesh(self, mesh_pts):
        time = rospy.Time.now()
        for i in range(mesh_pts.shape[0]):
            pose = mesh_pts[i]
            marker = self.build_marker(i, pose, time)
            self.array.markers.append(marker)

    def build_marker(self, id, pose, time):
        marker = Marker()
        marker.header.stamp = time
        marker.header.frame_id = "stapler_frame"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.id = id
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        return marker
    
    def publish_markers(self):
        self.pub.publish(self.array)

class SelectedMeshGraspBr(MeshGraspSelector):
    br: TransformBroadcaster
    targ_pose: np.ndarray
    array_publisher: MarkerArrayBuilder
    br_list: List[TransformBroadcaster]

    def __init__(self, object_name, tr_ang=10) -> None:
        super().__init__(object_name, tr_ang)
        self.br = TransformBroadcaster("/" + object_name + "_frame", "/target_grasp")
        self.array_publisher = MarkerArrayBuilder()

    def find_targ_pose(self):
        self.targ_pose = self.select_grasp()
        self.array_publisher.build_array_from_mesh(self.mesh_pts)

    def print_line(self):
        print(self.sel_npts, ", ", self.best_score)

    def build_all_br(self):
        self.br_list = []
        for i in range(len(self.grasp_candidates_poses)):
            br_i = TransformBroadcaster("/stapler_frame", "/" + str(i))
            self.br_list.append(br_i)

    def broadcast_all(self):
        for i in range(len(self.grasp_candidates_poses)):
            br = self.br_list[i]
            pose = self.grasp_candidates_poses[i]
            br.broadcast_transform(pose)

    def broadcast_target_mesh_grasp(self):
        self.br.broadcast_transform(self.targ_pose)
        self.array_publisher.publish_markers()
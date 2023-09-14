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
    grasp_candidates: List[np.ndarray]
    obj_pose: np.ndarray
    cam_pose: np.ndarray

    obj_pose_listener: TfListener
    cam_pose_lilstener: TfListener

    tr_ang: float
    operator: VectorOperator
    eef_scene: EEFScene
    col_pcd: PointCloud

    trans_msh_pts: np.ndarray

    def __init__(self, object_name, tr_ang=10) -> None:
        self.load_mesh(object_name)
        self.obj_pose_listener = TfListener(object_name + "_frame")
        self.cam_pose_lilstener = TfListener("camera_frame")
        self.tr_ang = tr_ang*np.pi/180
        self.operator = VectorOperator()
        self.grasp_candidates = []

        self.col_pcd = PointCloud(object_name + "_frame")
        self.eef_scene = EEFScene()

    def select_grasp(self):
        self.set_obj_and_cam_poses()
        self.select_cands_by_ang()
        self.col_pcd.set_new_pcd_wrt_obj(voxel_size=0)
        
        sel_pose = None
        best_score = math.inf
        for pt in self.grasp_candidates:
            pose = self.build_random_oriented_pose(pt)
            n_pts, score = self.eef_scene.compute_collision_pts_and_score(pose, self.col_pcd)
            if n_pts == 0 and score < best_score:
                best_score = score
                sel_pose = pose
        print("The selected pose is at: ", sel_pose)
        return sel_pose
            
    def set_obj_and_cam_poses(self):
        self.cam_pose_lilstener.listen()
        self.obj_pose_listener.listen()

        self.cam_pose = self.cam_pose_lilstener.get_np_frame()
        self.obj_pose = self.obj_pose_listener.get_np_frame()
    
    def select_cands_by_ang(self):
        self.trans_msh_pts = self.transform_mesh(self.obj_pose)
        obj_cam_vec = self.cam_pose[:3, 3] - self.obj_pose[:3, 3]

        min_ang = math.inf
        for i in range(self.trans_msh_pts.shape[0]):
            pt = self.trans_msh_pts[i]
            obj_pt_vec = pt - self.obj_pose[:3, 3].T
            angle = self.operator.compute_angle(init_vect=obj_cam_vec, end_vect=obj_pt_vec)
            if angle < self.tr_ang:
                if angle < min_ang: min_ang = angle
                self.grasp_candidates.append(pt)
        print(min_ang*180/math.pi)

    def build_random_oriented_pose(self, point:np.ndarray):
        Rt_pose = np.eye(4)
        Rt_pose[:3, 3] = point

        vec = point - self.obj_pose[:3, 3].T
        w = np.random.random()*np.pi/2
        if np.random.random() > 0.5:
            w *= -1 
        
        quat = np.append(vec, np.array([w]), axis=0)
        Rt_pose[:3, :3] = R.from_quat(quat).as_matrix()
        return Rt_pose.copy()
    
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
        marker.header.frame_id = "panda_link0"
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

    def __init__(self, object_name, tr_ang=10) -> None:
        super().__init__(object_name, tr_ang)
        self.br = TransformBroadcaster("/panda_link0", "/target_grasp")
        self.array_publisher = MarkerArrayBuilder()

    def find_targ_pose(self):
        self.targ_pose = self.select_grasp()
        self.array_publisher.build_array_from_mesh(self.trans_msh_pts)

    def broadcast_target_mesh_grasp(self):
        self.br.broadcast_transform(self.targ_pose)
        self.array_publisher.publish_markers()
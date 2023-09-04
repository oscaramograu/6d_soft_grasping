import open3d as o3d
import numpy as np
from impose_grasp.lib.tf_listener import TfListener

from ctypes import * # convert float to uint32
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from impose_grasp.lib.geometry import invert_homogeneous

from impose_grasp.lib.frame_builder import FrameBuilder
from impose_grasp.models.cameras.base_camera import CamFrame

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

class PointCloud(FrameBuilder):
    def __init__(self, target_frame) -> None:
        super().__init__()
        self.target = target_frame

        self.obstruction_pcl: o3d.t.geometry.PointCloud = None
        self.voxel_size = 0

        self.listener = TfListener(
            target_frame= "camera_frame", base_frame=target_frame)

    def set_new_pcd_wrt_obj(self, voxel_size):
        """
        Grabs an RGBD frame to build a pointcloud using open3d.

        Sets the  the attribute obstruction pointcloud in which the points 
        are in object coordinates.
        """
        self.voxel_size = voxel_size
        frame = None
        while frame is None:
            frame = self.get_actual_frame() 

        self._build_obstr_wrt_obj(frame)

    def get_pcd(self):
        return self.obstruction_pcl.cpu().clone()

    def _build_obstr_wrt_obj(self, frame: CamFrame):
        """
        Builds the attribute obstruction pointcloud in which the points 
        are in object coordinates. It then selects only the pts which
        are within 15cm around the target object.
        """
        stride_ = 4
        remove_tolerance = 0.01
        persistency = 1.0
        gpu = o3d.core.Device('CUDA:0')

        rgb = o3d.t.geometry.Image(o3d.core.Tensor.from_numpy(
            frame.rgb))#.to(gpu)
        depth = o3d.t.geometry.Image(o3d.core.Tensor.from_numpy(
            frame.depth))#.to(gpu)
        rgbd = o3d.t.geometry.RGBDImage(rgb, depth)
        intr_mat = o3d.core.Tensor(frame.intrinsic)
        pcd = o3d.t.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intr_mat, depth_scale=1.0, stride=max(stride_, 1))

        if self.voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        self._set_target_wrt_cam()
        pcd.transform(self.target_pose)

        pcd = self.select_pts_in_range(pcd, 0.3)
        
        self.obstruction_pcl = pcd.cpu().clone()

    def select_pts_in_range(self, pcd_, range):
        pcd = pcd_.cpu().clone()
        np_pts = pcd.point.positions.numpy()
        euclidean_norms = np.linalg.norm(np_pts, axis=1)
        pcd = pcd.select_by_mask(o3d.core.Tensor(
            euclidean_norms < range))
        return pcd.cpu().clone()
    
    def get_pcd_in_ros(self, open3d_cloud, frame_id = None):
        """
        Given a open3d pointcloud object, a ros pointcloud is returned 
        wrt the given frame id.
        """
        header = Header()
        header.stamp = rospy.Time.now()

        if frame_id is None:
            header.frame_id = self.target
        else:
            header.frame_id = frame_id

        points=np.asarray(open3d_cloud.point.positions)
        fields=FIELDS_XYZ
        cloud_data=points

        item_func = np.frompyfunc(lambda tensor: tensor.item(), 1, 1)

        cloud_data = item_func(cloud_data)        
        return pc2.create_cloud(header, fields, cloud_data)
    
    def _set_target_wrt_cam(self):
        """
        Sets the transformation from the object frame to the camera frame.
        Used to convert the pointcloud from camera to object coordinates.
        """
        self.listener.listen()
        self.target_pose = self.listener.get_np_frame()

    def get_pcd_wrt_target(self, target_g_pose: np.ndarray):
        """
        Returns a new pointcloud with respect to a given target grasp pose.
        """
        pcd = self.obstruction_pcl.cpu().clone()
        pcd.transform(invert_homogeneous(target_g_pose))
        return pcd.cpu().clone()
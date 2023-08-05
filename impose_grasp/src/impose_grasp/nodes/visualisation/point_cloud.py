import open3d as o3d
import numpy as np
from impose_grasp.lib.tf_listener import TfListener

from ctypes import * # convert float to uint32
import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

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

    def build_pcd_wrt_obj(self, voxel_size):
        self.voxel_size = voxel_size
        frame = None
        while frame is None:
            frame = self.get_actual_frame() 

        self._build_obstr_wrt_obj(frame)

    def get_pcd(self):
        return self.obstruction_pcl.cpu().clone()

    def _build_obstr_wrt_obj(self, frame: CamFrame):
        """
        Builds the obstruction point cloud from the camera frame. 
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

        self.set_target_wrt_cam()
        pcd.transform(self.target_pose)     

        self.obstruction_pcl = pcd.cpu().clone()
    
    def get_pcd_in_ros(self):
        open3d_cloud = self.obstruction_pcl.cpu().clone()
        # Set "header"
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.target

        # Set "fields" and "cloud_data"
        points=np.asarray(open3d_cloud.point.positions)
        fields=FIELDS_XYZ
        cloud_data=points

        # Create a ufunc using np.frompyfunc that applies the item() method to each element
        item_func = np.frompyfunc(lambda tensor: tensor.item(), 1, 1)

        # Use the ufunc to retrieve the float values
        cloud_data = item_func(cloud_data)

        print(cloud_data[0])
        return pc2.create_cloud(header, fields, cloud_data)
    
    def set_target_wrt_cam(self):
        self.listener.listen_tf()
        self.target_pose = self.listener.get_np_frame()